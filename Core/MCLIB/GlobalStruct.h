/*
 * GlobalStruct.h
 *
 *  Created on: Mar 17, 2024
 *      Author: r720r
 */

#ifndef MCLIB_GLOBALSTRUCT_H_
#define MCLIB_GLOBALSTRUCT_H_

struct SensorData {
    float electAngle;   // 電気角
    float electAngVelo;
    float Iuvw[3];      // 3相電流
    float Vdc;          // 直流電圧
    float twoDivVdc;    // 直流電圧の2分の1
};

struct VectorControlData {
    float Idq[2];   // 電気角
    float Vdq[2];      // 3相電流
    float Vdq_i[2];
};


#endif /* MCLIB_GLOBALSTRUCT_H_ */
