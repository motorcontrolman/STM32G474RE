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
static void slctElectAngleFromPosMode(uint8_t posMode, struct SensorData *sensData);
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
		//sElectAngVeloRef = 2000.0f * gVolume;
		if(gButton1 == 1)
		{
			sElectAngVeloRef = 300.0f;
			gRateLimit(sElectAngVeloRef, 500.0f, CARRIERCYCLE, &sElectAngVeloRefRateLimit);
		}
		else
			sElectAngVeloRefRateLimit = 0;

		// For Sensor Drive
		slctPosMode(gElectFreq, &sPosMode);
		slctDrvMode(gElectFreq, &sDrvMode);

		// For Sensorless Drive
		//slctPosModeForSensorless(sSensData.electAngVelo, &sPosMode);
		//slctDrvModeForSensorless(sSensData.electAngVelo, &sDrvMode);
	}

	slctElectAngleFromPosMode(sPosMode, &sSensData);
	slctCntlFromDrvMode(sDrvMode, sSensData, &sVectorControlData, sDuty, sOutputMode);

	writeOutputMode(sOutputMode);
	writeDuty(sDuty);
	//writeDuty8(sDuty);

}

void slctPosMode(float electFreq, uint8_t* posMode){

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

	if(*posMode != POSMODE_SENSORLESS){
		if (electAngVelo > ELECTANGVELO_FREERUN2SENSORLESS)
			*posMode = POSMODE_SENSORLESS;
		else
			*posMode = POSMODE_FREERUN;
	}
	else if(*posMode == POSMODE_SENSORLESS){
		if (electAngVelo < ELECTANGVELO_SENSORLESS2FREERUN)
			*posMode = POSMODE_FREERUN;
		else
			*posMode = POSMODE_SENSORLESS;
	}
}

static void slctDrvModeForSensorless(float electAngVelo, uint8_t* drvMode){

	if(*drvMode != DRVMODE_VECTORCONTROL){
		if (electAngVelo > ELECTANGVELO_OPENLOOP2VECTORCONTROL)
			*drvMode = DRVMODE_VECTORCONTROL;
		else
			*drvMode = DRVMODE_OPENLOOP;
	}
	else if(*drvMode == DRVMODE_VECTORCONTROL){
		if (electAngVelo < ELECTANGVELO_VECTORCONTROL2OPENLOOP)
			*drvMode = DRVMODE_OPENLOOP;
		else
			*drvMode = DRVMODE_VECTORCONTROL;
	}
}

static void slctElectAngleFromPosMode(uint8_t posMode, struct SensorData *sensData){
	uint8_t flgInit;
	uint8_t flgPLL;
	float electAngle;
	float electAngVelo;

	switch(posMode){
	case POSMODE_STOP:
		sensData->electAngle = 0.0f;
		sensData->electAngVelo = 0.0f;
		break;

	case POSMODE_FREERUN:
		sensData->electAngVelo = sElectAngVeloRefRateLimit;
		sElectAngleFreerun += sElectAngVeloRefRateLimit * CARRIERCYCLE ;
		sensData->electAngle = gfWrapTheta(sElectAngleFreerun);

		// For Sensorless Init
		flgInit = 0;
		calcElectAngleEstimate(flgInit, sSensData, sVectorControlData, &sElectAngleEstimateData);
		break;
	case POSMODE_HALL:
		flgPLL = 0;
		calcElectAngle(gHall, gElectFreq, flgPLL, &electAngle, &electAngVelo);
		sensData->electAngle = electAngle;
		sensData->electAngVelo = electAngVelo;
		break;
	case POSMODE_HALL_PLL:
		flgPLL = 1;
		calcElectAngle(gHall, gElectFreq, flgPLL, &electAngle, &electAngVelo);
		sensData->electAngle = electAngle;
		sensData->electAngVelo = electAngVelo;
		break;
	case POSMODE_SENSORLESS:
		flgInit = 1;
		calcElectAngleEstimate(flgInit, sSensData, sVectorControlData, &sElectAngleEstimateData);
		sensData->electAngle = sElectAngleEstimateData.electAngleEstimate;
		sensData->electAngVelo = sElectAngleEstimateData.electAngVeloEstimate;
		break;
	default:
		sensData->electAngle = 0.0f;
		sensData->electAngVelo = 0.0f;
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

	Idq_ref[1] = IQREFMAX * gButton1;// * gVolume;

	switch(drvMode){
		case DRVMODE_OFFDUTY:
			gOffDuty(Duty, outputMode);
			break;
		case DRVMODE_OPENLOOP:
			//VamRef = sSensData.Vdc * SQRT3DIV2_DIV2 * ( 0.1f + 0.3f * sElectAngVeloRefRateLimit*0.0005f);//sElectAngVeloRefRateLimit * 0.001 );
			VamRef = sSensData.Vdc * SQRT3DIV2_DIV2 * 0.5f * gButton1;;
			OpenLoopTasks(VamRef, sensData, vectorControlData, Duty, outputMode);
			break;
		case DRVMODE_VECTORCONTROL:
			VectorControlTasks(Idq_ref, sensData, vectorControlData, Duty, outputMode);
			break;
		default :
			gOffDuty(Duty, outputMode);
	}
}

