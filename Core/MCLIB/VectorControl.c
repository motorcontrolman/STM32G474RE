/*
 * VectorControl.c
 *
 *  Created on: Jan 7, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "math.h"
#include "GeneralFunctions.h"
#include "GlobalConstants.h"
#include "GlobalStruct.h"
#include "VectorControl.h"
#include "ControlFunctions.h"

// for Debug
#include "SignalReadWrite.h"

static float sIab[3];
static float sIdq[2];
static float sIq_LPF = 0;
static float sIq_ref_LPF = 0;
static float sIdq_ref_1000[2];
static float sIdq_1000[2];
static float sVdq[2];
static float sVab[2];
static float sVuvw[3];
static float sVamp;
static float sMod;
static float sEdq[2];

static float sElectAngleEstimate = 0.0f;
static float sIntegral_ElectAngleErr_Ki = 0.0f;
static float sElectAngVeloEstimate;
static float sElectAngleErr;

static inline void uvw2ab(float *uvw, float *ab);
static inline void ab2uvw(float *ab, float *uvw);
static inline void ab2dq(float theta, float *ab, float *dq);
static inline void dq2ab(float theta, float *dq, float *ab);
static inline float calcAmpFromVect(float* Vect);
static inline float calcModFromVamp(float Vamp, float twoDivVdc);
static inline void Vuvw2Duty(float twoDivVdc, float *Vuvw, float *Duty);
static inline void Vuvw2DutyforOpenWinding(float twoDivVdc, float *Vuvw, float *Duty);
static inline void CurrentFbControl(float *Igd_ref, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Vamp);
static inline float FluxObserver(float* Igd, float* Vgd, float electAngVelo, float* Egd);
static inline void calcElectAngleEstimate(uint8_t flgPLL, float electAngle, float electAngVelo, float *electAngleEstimate, float *electAngVeloEstimate);

void VectorControlTasks(float *Idq_ref, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode){

	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;

	float theta;
	float omega;


	//calcElectAngleEstimate(flgPLL, electAngle, electAngVelo, &sElectAngleEstimate, &sElectAngVeloEstimate);

	//theta = sElectAngleEstimate;
	//omega = sElectAngVeloEstimate;
	theta = sensData.electAngle;
	omega = sensData.electAngVelo;


	uvw2ab(sensData.Iuvw, sIab);
	ab2dq(sensData.electAngle, sIab, vectorControlData->Idq);

	gLPF(Idq_ref[1], 62.8f, CARRIERCYCLE, &sIq_ref_LPF);
	Idq_ref[1] = sIq_ref_LPF; // zanteisyori
	CurrentFbControl(Idq_ref, sensData, vectorControlData, &sVamp);
	sMod = calcModFromVamp(sVamp, sensData.twoDivVdc);
	dq2ab(sensData.electAngle, vectorControlData->Vdq, sVab);
	ab2uvw(sVab, sVuvw);

	Vuvw2Duty(sensData.twoDivVdc, sVuvw, Duty);


	sIdq_ref_1000[0] = Idq_ref[0] * 1000.0f;
	sIdq_ref_1000[1] = Idq_ref[1] * 1000.0f;
	sIdq_1000[0] = vectorControlData->Idq[0] * 1000.0f;
	sIdq_1000[1] = vectorControlData->Idq[1] * 1000.0f;

	gLPF(vectorControlData->Idq[1], 125.6f, CARRIERCYCLE, &sIq_LPF);

}

void OpenLoopTasks(float VamRef, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode){
	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;

	uvw2ab(sensData.Iuvw, sIab);
	ab2dq(sensData.electAngle, sIab, vectorControlData->Idq);
	vectorControlData->Vdq[0] = 0.0f;
	vectorControlData->Vdq[1] = VamRef;
	vectorControlData->Vdq_i[0] = vectorControlData->Vdq[0];
	vectorControlData->Vdq_i[1] = vectorControlData->Vdq[1];

	dq2ab(sensData.electAngle, vectorControlData->Vdq, sVab);
	ab2uvw(sVab, sVuvw);
	Vuvw2Duty(sensData.twoDivVdc, sVuvw, Duty);

	sIdq_1000[0] = sIdq[0] * 1000.0f;
	sIdq_1000[1] = sIdq[1] * 1000.0f;
}

static void uvw2ab(float* uvw, float* ab){
	ab[0] = SQRT_2DIV3 * ( uvw[0] - 0.5f * uvw[1] - 0.5f * uvw[2] );
	ab[1] = SQRT_2DIV3 * ( SQRT3_DIV3 * uvw[1] - SQRT3_DIV3 * uvw[2] );
	ab[2] = SQRT_1DIV3 * ( uvw[0] + uvw[1] + uvw[2] );
}

static void ab2uvw(float* ab, float* uvw){
	uvw[0] = SQRT_2DIV3 * ab[0];
	uvw[1] = SQRT_2DIV3 * ( -0.5f * ab[0] + SQRT3_DIV3 * ab[1] );
	uvw[2] = - uvw[0] - uvw[1];
}

static void ab2dq(float theta, float* ab, float* dq){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	dq[0] = ab[0] * cosTheta + ab[1] * sinTheta;
	dq[1] = - ab[0] * sinTheta + ab[1] * cosTheta;
}

static float calcAmpFromVect(float* Vect){
	float amp;
	float sumOfSquares;

	sumOfSquares = Vect[0] * Vect[0] + Vect[1] * Vect[1];
	amp = sqrtf(sumOfSquares);
	return amp;
}

static float calcModFromVamp(float Vamp, float twoDivVdc){
	float mod;

	mod = Vamp * twoDivVdc * SQRT_2DIV3;
	return mod;
}

static void dq2ab(float theta, float* dq, float* ab){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	ab[0] = dq[0] * cosTheta - dq[1] * sinTheta;
	ab[1] = dq[0] * sinTheta + dq[1] * cosTheta;
}

static void Vuvw2Duty(float twoDivVdc, float* Vuvw, float* Duty){

	float max;
	float min;
	float vo;

	// third-harmonic injection
	max = Vuvw[0];
	if(Vuvw[1] > max)
		max = Vuvw[1];
	if(Vuvw[2] > max)
		max = Vuvw[2];

	min = Vuvw[0];
	if(Vuvw[1] < min)
		min = Vuvw[1];
	if(Vuvw[2] < min)
		min = Vuvw[2];

	vo = (max + min) * 0.5f;

	Vuvw[0] = Vuvw[0] - vo;
	Vuvw[1] = Vuvw[1] - vo;
	Vuvw[2] = Vuvw[2] - vo;



	Duty[0] = (Vuvw[0] * twoDivVdc);
	Duty[1] = (Vuvw[1] * twoDivVdc);
	Duty[2] = (Vuvw[2] * twoDivVdc);//-Duty[0] - Duty[1];

	Duty[0] = gUpperLowerLimit(Duty[0], DUTYUPPER, DUTYLOWER);
	Duty[1] = gUpperLowerLimit(Duty[1], DUTYUPPER, DUTYLOWER);
	Duty[2] = gUpperLowerLimit(Duty[2], DUTYUPPER, DUTYLOWER);

	//50% CENTER
	Duty[0] = Duty[0] * 0.5f + 0.5f;
	Duty[1] = Duty[1] * 0.5f + 0.5f;
	Duty[2] = Duty[2] * 0.5f + 0.5f;

}

static inline void Vuvw2DutyforOpenWinding(float twoDivVdc, float *Vuvw, float *Duty){



	Duty[0] = (Vuvw[0] * twoDivVdc * 0.5f);
	Duty[1] = (Vuvw[1] * twoDivVdc * 0.5f);
	Duty[2] = (Vuvw[2] * twoDivVdc * 0.5f);

	Duty[0] = gUpperLowerLimit(Duty[0], DUTYUPPER, DUTYLOWER);
	Duty[1] = gUpperLowerLimit(Duty[1], DUTYUPPER, DUTYLOWER);
	Duty[2] = gUpperLowerLimit(Duty[2], DUTYUPPER, DUTYLOWER);

}


static void CurrentFbControl(float *Igd_ref, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Vamp){
	float Ierr[2];
	float Kp;
	float Kig;
	float Kid;
	float VampLimit;
	float Vphase;
	float wc;

	wc = 10.0f * TWOPI;

	Kp = La * wc;
	Kig = Ra * wc * CARRIERCYCLE;
	Kid = Kig;

	Ierr[0] = Igd_ref[0] - vectorControlData->Idq[0];
	Ierr[1] = Igd_ref[1] - vectorControlData->Idq[1];

	vectorControlData->Vdq_i[0] += Kig * Ierr[0];
	vectorControlData->Vdq_i[1] += Kid * Ierr[1];

	vectorControlData->Vdq[0] = Kp * Ierr[0] + vectorControlData->Vdq_i[0];
	vectorControlData->Vdq[1] = Kp * Ierr[1] + vectorControlData->Vdq_i[1];// + Ke * electAngVelo;// + Vgd[1] + Kid * Ierr[1] + ;

	Vphase = atan2f(vectorControlData->Vdq[1], vectorControlData->Vdq[0]);

	*Vamp = calcAmpFromVect(vectorControlData->Vdq);

	VampLimit = sensData.Vdc * SQRT3DIV2_DIV2 * 1.15f;
	if( *Vamp > VampLimit ){
		vectorControlData->Vdq[0] = VampLimit * cosf(Vphase);
		vectorControlData->Vdq_i[0] = vectorControlData->Vdq[0];
		vectorControlData->Vdq[1] = VampLimit * sinf(Vphase);
		vectorControlData->Vdq_i[1] = vectorControlData->Vdq[1];// -  Ke * sensData.electAngVelo;

	}
}


static float FluxObserver(float* Igd, float* Vgd, float electAngVelo, float* Egd){
	float angleErr;
	//Egd[0] = Vgd[0] - Ra * Igd[0];
	//Egd[1] = Vgd[1] - Ra * Igd[1];
	Egd[0] = Vgd[0] - Ra * Igd[0] + La * electAngVelo * Igd[1];
	Egd[1] = Vgd[1] - Ra * Igd[1] - La * electAngVelo * Igd[0];
	angleErr = atan2f(-1.0f * sEdq[0], sEdq[1]); //推定q軸を基準とした実q軸との誤差を算出
	//arm_atan2_f32(Egd[0], Egd[1], &result);
	//Theta_est = atan2f(Egd[1], Egd[0]);
	return angleErr;
}

static void calcElectAngleEstimate(uint8_t flgPLL, float electAngle, float electAngVelo, float *electAngleEstimate, float *electAngVeloEstimate){
	float wc_PLL;
	float Kp_PLL;
	float Ki_PLL;
	float Ts_PLL;

	sElectAngleErr = FluxObserver(sIdq, sVdq, *electAngleEstimate, sEdq);

	if( flgPLL == 0){
		*electAngVeloEstimate = electAngVelo;
		*electAngleEstimate = electAngle;
		sIntegral_ElectAngleErr_Ki = electAngVelo;
	}
	else{

		// Calculate PLL Gain based on Electrical Angle Velocity
		wc_PLL = 50.0f * TWOPI;//sElectAngVeloEstimate * 0.5f;
		Ts_PLL = CARRIERCYCLE;
		Kp_PLL = wc_PLL;
		Ki_PLL = 0.2f * wc_PLL * wc_PLL * Ts_PLL;

		// Estimate Electrical Angle & Velocity using PLL
		*electAngleEstimate += (*electAngVeloEstimate) * CARRIERCYCLE;
		*electAngleEstimate = gfWrapTheta(*electAngleEstimate);

		// wrap Electrical Angle Err
		sElectAngleErr = gfWrapTheta(sElectAngleErr);

		//PLL
		*electAngVeloEstimate = cfPhaseLockedLoop(sElectAngleErr, Kp_PLL, Ki_PLL, &sIntegral_ElectAngleErr_Ki);

	}

}
