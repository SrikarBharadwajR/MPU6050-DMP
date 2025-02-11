#ifndef _MPU6050_6AXIS_MOTIONAPPS612_H_
#define _MPU6050_6AXIS_MOTIONAPPS612_H_

#include "DMP_Firmware.h"
#include "MPU6050.h"

extern uint8_t *dmpPacketBuffer;
extern uint16_t dmpPacketSize;

uint8_t dmpInitialize(void);
bool dmpPacketAvailable(void);

uint8_t dmpSetFIFORate(uint8_t fifoRate);
uint8_t dmpGetFIFORate(void);
uint8_t dmpGetSampleStepSizeMS(void);
uint8_t dmpGetSampleFrequency(void);
int32_t dmpDecodeTemperature(int8_t tempReg);

// Register callbacks after a packet of FIFO data is processed
// uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
// uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
uint8_t dmpRunFIFORateProcesses(void);

// Setup FIFO for various output
uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

// Get Fixed Point data from FIFO
uint8_t dmpGetAccel32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetAccel16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetAccelVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetQuaternion32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetQuaternion16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t *packet);
uint8_t dmpGet6AxisQuaternion32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGet6AxisQuaternion16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t *packet);
uint8_t dmpGetRelativeQuaternion32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetRelativeQuaternion16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t *packet);
uint8_t dmpGetGyro32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetGyro16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetGyroVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
uint8_t dmpGetLinearAccel32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetLinearAccel16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetLinearAccelVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetLinearAccelVectRaw(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t dmpGetLinearAccelInWorld32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetLinearAccelInWorld16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetLinearAccelInWorldVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetLinearAccelInWorldVectQuat(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t dmpGetGyroAndAccelSensor32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetGyroAndAccelSensor16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetGyroAndAccelSensorVect(VectorInt16 *g, VectorInt16 *a, const uint8_t *packet);
uint8_t dmpGetGyroSensor32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetGyroSensor16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetGyroSensorVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetControlData32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetTemperature32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetGravity32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetGravity16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetGravityVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetGravityVectQuat(VectorFloat *v, Quaternion *q);
uint8_t dmpGetUnquantizedAccel32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetUnquantizedAccel16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetUnquantizedAccelVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetQuantizedAccel32bit(int32_t *data, const uint8_t *packet);
uint8_t dmpGetQuantizedAccel16bit(int16_t *data, const uint8_t *packet);
uint8_t dmpGetQuantizedAccelVect(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t *packet);
uint8_t dmpGetEIS(int32_t *data, const uint8_t *packet);

uint8_t dmpGetEuler(float *data, Quaternion *q);
uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

// Get Floating Point data from FIFO
uint8_t dmpGetAccelFloat(float *data, const uint8_t *packet);
uint8_t dmpGetQuaternionFloat(float *data, const uint8_t *packet);

uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);

uint8_t dmpSetFIFOProcessedCallback(void (*func)(void));

uint8_t dmpInitFIFOParam(void);
uint8_t dmpCloseFIFO(void);
uint8_t dmpSetGyroDataSource(uint8_t source);
uint8_t dmpDecodeQuantizedAccel(void);
uint32_t dmpGetGyroSumOfSquare(void);
uint32_t dmpGetAccelSumOfSquare(void);
void dmpOverrideQuaternion(long *q);
uint16_t dmpGetFIFOPacketSize(void);
uint8_t dmpGetCurrentFIFOPacket(uint8_t *data);    // overflow proof

#endif                                             /* _MPU6050_6AXIS_MOTIONAPPS612_H_ */
