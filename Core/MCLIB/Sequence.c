/*
 * Sequence.c
 *
 *  Created on: Sep 2, 2023
 *      Author: r720r
 */

#include "GlobalVariables.h"
#include <stdint.h>
#include "main.h"
#include "SignalReadWrite.h"
#include "GeneralFunctions.h"
#include "GlobalConstants.h"
#include "GlobalStruct.h"
#include "GlobalVariables.h"
#include "Sequence.h"
#include "SixsStep.h"
#include "VectorControl.h"

static uint8_t sPosMode;
static uint8_t sDrvMode;
static uint16_t sInitCnt = 0;
static float sElectAngle = 0;
static float sElectAngleFreerun = 0;
static float sElectAngVelo;
static float sElectAngVeloRef = 0;
static float sElectAngVeloRefRateLimit = 0;
static int8_t sOutputMode[3];
static float sDuty[3];
static struct SensorData sSensData;
static struct VectorControlData sVectorControlData;
static struct ElectAngleEstimateData sElectAngleEstimateData = {0.0f, 0.0f, 0.0f};
static float sId_ref_i = 0;

static void slctPosMode(float electFreq, uint8_t* posMode);
static void slctDrvMode(float electFreq, uint8_t* drvMode);
static void slctPosModeForSensorless(float electAngVelo, uint8_t* posMode);
static void slctDrvModeForSensorless(float electAngVelo, uint8_t* drvMode);
static void slctElectAngleFromPosMode(uint8_t posMode, float *electAngle, float *electAngVelo);
static void slctCntlFromDrvMode(uint8_t drvMode, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode);

void Sequence(void){

	//read IO signals
	gButton1 = readButton1();
	gVolume = readVolume();

	readHallSignal(gHall);
	readElectFreqFromHallSignal(&gElectFreq);

	readCurrent(gIuvw_AD, sSensData.Iuvw);
	sSensData.Vdc = readVdc();
	sSensData.twoDivVdc = gfDivideAvoidZero(2.0f, sSensData.Vdc, 1.0f);;

	if(sInitCnt < 500){
		sInitCnt++;
		sPosMode = POSMODE_HALL;
		sDrvMode = DRVMODE_OFFDUTY;
		sElectAngVeloRefRateLimit = 0;
	}
	else {
		// For Sensor Drive
		//slctPosMode(gElectFreq, &sPosMode);
		//slctDrvMode(gElectFreq, &sDrvMode);

		// For Sensorless Drive
		sElectAngVeloRef = 2000.0f * gVolume;//1000.0f * gButton1;//2000.0f * gVolume;
		gRateLimit(sElectAngVeloRef, 200.0f, CARRIERCYCLE, &sElectAngVeloRefRateLimit);
		//sElectAngVeloRefRateLimit = gButton1 * sElectAngVeloRefRateLimit;

		slctPosModeForSensorless(sElectAngVeloRefRateLimit, &sPosMode);
		slctDrvModeForSensorless(sElectAngVeloRefRateLimit, &sDrvMode);
	}

	slctElectAngleFromPosMode(sPosMode, &sElectAngle, &sElectAngVelo);

	sSensData.electAngle = sElectAngle;
	sSensData.electAngVelo = sElectAngVelo;


	slctCntlFromDrvMode(sDrvMode, sSensData, &sVectorControlData, sDuty, sOutputMode);

	writeOutputMode(sOutputMode);
	writeDuty(sDuty);
	//writeDuty8(sDuty);

}

void slctPosMode(float electFreq, uint8_t* posMode){

	*posMode = POSMODE_FREERUN;

	if(*posMode != POSMODE_HALL_PLL){
		if (electFreq > ELECTFREQ_VALIDPLL)
			*posMode = POSMODE_HALL_PLL;
		else
			*posMode = POSMODE_HALL;
	}
	else if(*posMode == POSMODE_HALL_PLL){
		if (electFreq < ELECTFREQ_INVALIDPLL)
			*posMode = POSMODE_HALL;
		else
			*posMode = POSMODE_HALL_PLL;
	}

}

void slctDrvMode(float electFreq, uint8_t* drvMode){

	if(*drvMode != DRVMODE_VECTORCONTROL){
		if (electFreq > ELECTFREQ_OPENLOOP2VECTORCONTROL)
			*drvMode = DRVMODE_VECTORCONTROL;
		else
			*drvMode = DRVMODE_OPENLOOP;
	}
	else if(*drvMode == DRVMODE_VECTORCONTROL){
		if (electFreq < ELECTFREQ_VECTORCONTROL2OPENLOOP)
			*drvMode = DRVMODE_OPENLOOP;
		else
			*drvMode = DRVMODE_VECTORCONTROL;
	}
}

static void slctPosModeForSensorless(float electAngVelo, uint8_t* posMode){
	if (electAngVelo < 200.0f)
		*posMode = POSMODE_FREERUN;
	else
		*posMode = POSMODE_SENSORLESS;
}

static void slctDrvModeForSensorless(float electAngVelo, uint8_t* drvMode){
	if (electAngVelo < 400.0f)
	//if(gButton1 == 0)
		*drvMode = DRVMODE_OPENLOOP;
	else
		*drvMode = DRVMODE_VECTORCONTROL;
}

void slctElectAngleFromPosMode(uint8_t posMode, float *electAngle, float *electAngVelo){
	uint8_t flgInit;
	uint8_t flgPLL;

	switch(posMode){
	case POSMODE_STOP:
		*electAngle = 0.0f;
		*electAngVelo = 0.0f;
		sElectAngVeloRef = 0.0f;
		break;

	case POSMODE_FREERUN:
		*electAngVelo = sElectAngVeloRefRateLimit;
		sElectAngleFreerun += sElectAngVeloRefRateLimit * CARRIERCYCLE ;
		*electAngle = gfWrapTheta(sElectAngleFreerun);

		// For Sensorless Init
		flgInit = 0;
		calcElectAngleEstimate(flgInit, sVectorControlData, &sElectAngleEstimateData);
		sElectAngleEstimateData.electAngleEstimate = *electAngle;
		sElectAngleEstimateData.electAngVeloEstimate = *electAngVelo;
		break;
	case POSMODE_HALL:
		flgPLL = 0;
		calcElectAngle(gHall, gElectFreq, flgPLL, electAngle, electAngVelo);
		break;
	case POSMODE_HALL_PLL:
		flgPLL = 1;
		calcElectAngle(gHall, gElectFreq, flgPLL, electAngle, electAngVelo);
		break;
	case POSMODE_SENSORLESS:
		flgInit = 1;
		calcElectAngleEstimate(flgInit, sVectorControlData, &sElectAngleEstimateData);
		*electAngle = sElectAngleEstimateData.electAngleEstimate;
		*electAngVelo = sElectAngleEstimateData.electAngVeloEstimate;
		break;
	default:
		*electAngle = 0;
		*electAngVelo = 0;
		break;
	}
}

void slctCntlFromDrvMode(uint8_t drvMode, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode){

	float Idq_ref[2];
	float VamRef;
	float ModRef = 1.13;
	float ModErr;

	//Idq_ref[0] = 0.0f;
	ModErr = ModRef - vectorControlData->Mod;
	sId_ref_i = sId_ref_i + 0.0003 * ModErr;

	if( sId_ref_i > 0)
			sId_ref_i = 0;
	if( sId_ref_i < -1.0f)
				sId_ref_i = -1.0f;

	Idq_ref[0] = sId_ref_i;

	Idq_ref[1] = IQREFMAX * gVolume;

	switch(drvMode){
		case DRVMODE_OFFDUTY:
			gOffDuty(Duty, outputMode);
			break;
		case DRVMODE_OPENLOOP:
			VamRef = sSensData.Vdc * SQRT3DIV2_DIV2 * ( 0.3f + 0.7f * gVolume);//sElectAngVeloRefRateLimit * 0.001 );
			OpenLoopTasks(VamRef, sensData, vectorControlData, Duty, outputMode);
			break;
		case DRVMODE_VECTORCONTROL:
			VectorControlTasks(Idq_ref, sensData, vectorControlData, Duty, outputMode);
			break;
		default :
			gOffDuty(Duty, outputMode);
	}
}

