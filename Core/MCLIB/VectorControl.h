#include <stdint.h>
#include "math.h"
#include "GlobalStruct.h"

void VectorControlTasks(float *Idq_ref, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode);
void OpenLoopTasks(float VamRef, struct SensorData sensData, struct VectorControlData *vectorControlData, float *Duty, int8_t* outputMode);
void calcElectAngleEstimate(uint8_t flgInit, struct SensorData sensData, struct VectorControlData vectorControlData, struct ElectAngleEstimateData *electAngleEstimateData);
