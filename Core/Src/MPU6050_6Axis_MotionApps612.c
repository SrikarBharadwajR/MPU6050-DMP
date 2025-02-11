// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS612

#include "MPU6050_6Axis_MotionApps612.h"

#define strcpy_P(dest, src) strcpy((dest), (src))
#define strcat_P(dest, src) strcat((dest), (src))
#define strcmp_P(a, b) strcmp((a), (b))

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))

// #define DEBUG

/* ================================================================ *
 | Default MotionApps v6.12 28-byte FIFO packet structure:           |
 |                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  |
 |                                                                  |
 | [GYRO X][GYRO Y][GYRO Z][ACC X ][ACC Y ][ACC Z ]					|
 |  16  17  18  19  20  21  22  23  24  25  26  27					|
 * ================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
// *** this is a capture of the DMP Firmware V6.1.2 after all the messy changes were made so we can just load it
// this divisor is pre configured into the above image and can't be modified at this time.
#ifndef MPU6050_DMP_FIFO_RATE_DIVISOR
#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01    // The New instance of the Firmware has this as the default
#endif
uint8_t *dmpPacketBuffer;
uint16_t dmpPacketSize;

// this is the most basic initialization I can create. with the intent that we access the register bytes as few times as
// needed to get the job done. for detailed descriptions of all registers and there purpose google "MPU-6000/MPU-6050
// Register Map and Descriptions"
uint8_t dmpInitialize(void)
{    // Lets get it over with fast Write everything once and set it up nicely
	uint8_t val;
	uint16_t ival;
	// Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and Descriptions" page 41
	writeBit(mpu.devAddr, 0x6B, 7, (val = 1), NULL); // PWR_MGMT_1: reset with 100ms delay
	HAL_Delay(100);
	writeBits(mpu.devAddr, 0x6A, 2, 3, (val = 0b111), NULL); // full SIGNAL_PATH_RESET: with another 100ms delay
	HAL_Delay(100);
	val = 0x01;
	writeBytes(mpu.devAddr, 0x6B, 1, &(val), NULL); // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
	val = 0x00;
	writeBytes(mpu.devAddr, 0x38, 1, &(val), NULL); // 0000 0000 INT_ENABLE: no Interrupt
	val = 0x00;
	writeBytes(mpu.devAddr, 0x23, 1, &(val), NULL); // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	val = 0x00;
	writeBytes(mpu.devAddr, 0x1C, 1, &(val), NULL); // 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
	val = 0x80;
	writeBytes(mpu.devAddr, 0x37, 1, &(val), NULL); // 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is
	// active low. and interrupt status bits are cleared on any read
	val = 0x01;
	writeBytes(mpu.devAddr, 0x6B, 1, &(val), NULL); // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
	val = 0x04;
	writeBytes(mpu.devAddr, 0x19, 1, &(val), NULL); // 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz (
													// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
	val = 0x01;
	writeBytes(mpu.devAddr, 0x1A, 1, &(val), NULL); // 0000 0001 CONFIG: Digital Low Pass Filter (DLPF)
													// Configuration 188HZ  //Im betting this will be the beat
	if (!writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true))
		return 1;                                         //   Fail
														  // Loads the DMP image into the MPU6050 Memory // Should Never
	ival = 0x0400;
	writeWords(mpu.devAddr, 0x70, 1, &(ival), NULL); // DMP Program Start Address
	HAL_Delay(1);
	val = 0x18;
	writeBytes(mpu.devAddr, 0x1B, 1, &(val), NULL); // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
	val = 0xC0;
	writeBytes(mpu.devAddr, 0x6A, 1, &(val), NULL); // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
	val = 0x02;
	writeBytes(mpu.devAddr, 0x38, 1, &(val), NULL); // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	writeBit(mpu.devAddr, 0x6A, 2, 1, NULL); // Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A
											 // first and only alters 1 bit and then saves the byte)

	setDMPEnabled(false); // disable DMP for compatibility with the MPU6050 library
	/*
	 dmpPacketSize += 16;//DMP_FEATURE_6X_LP_QUAT
	 dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_ACCEL
	 dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_GYRO
	 */
	dmpPacketSize = 28;
	return 0;
}

bool dmpPacketAvailable(void)
{
	return getFIFOCount() >= dmpGetFIFOPacketSize();
}

// uint8_t dmpSetFIFORate(uint8_t fifoRate);
// uint8_t dmpGetFIFORate();
// uint8_t dmpGetSampleStepSizeMS();
// uint8_t dmpGetSampleFrequency();
// int32_t dmpDecodeTemperature(int8_t tempReg);

// uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
// uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
// uint8_t dmpRunFIFORateProcesses();

// uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t dmpGetAccel32bit(int32_t *data, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (((uint32_t) packet[16] << 8) | packet[17]);
	data[1] = (((uint32_t) packet[18] << 8) | packet[19]);
	data[2] = (((uint32_t) packet[20] << 8) | packet[21]);
	return 0;
}
uint8_t dmpGetAccel16bit(int16_t *data, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (packet[16] << 8) | packet[17];
	data[1] = (packet[18] << 8) | packet[19];
	data[2] = (packet[20] << 8) | packet[21];
	return 0;
}
uint8_t dmpGetAccelVect(VectorInt16 *v, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	v->x = (packet[16] << 8) | packet[17];
	v->y = (packet[18] << 8) | packet[19];
	v->z = (packet[20] << 8) | packet[21];
	return 0;
}
uint8_t dmpGetQuaternion32bit(int32_t *data, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (((uint32_t) packet[0] << 24) | ((uint32_t) packet[1] << 16)
			| ((uint32_t) packet[2] << 8) | packet[3]);
	data[1] = (((uint32_t) packet[4] << 24) | ((uint32_t) packet[5] << 16)
			| ((uint32_t) packet[6] << 8) | packet[7]);
	data[2] = (((uint32_t) packet[8] << 24) | ((uint32_t) packet[9] << 16)
			| ((uint32_t) packet[10] << 8) | packet[11]);
	data[3] = (((uint32_t) packet[12] << 24) | ((uint32_t) packet[13] << 16)
			| ((uint32_t) packet[14] << 8) | packet[15]);
	return 0;
}
uint8_t dmpGetQuaternion16bit(int16_t *data, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = ((packet[0] << 8) | packet[1]);
	data[1] = ((packet[4] << 8) | packet[5]);
	data[2] = ((packet[8] << 8) | packet[9]);
	data[3] = ((packet[12] << 8) | packet[13]);
	return 0;
}
uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	int16_t qI[4];
	uint8_t status = dmpGetQuaternion16bit(qI, packet);
	if (status == 0)
	{
		q->w = (float) qI[0] / 16384.0f;
		q->x = (float) qI[1] / 16384.0f;
		q->y = (float) qI[2] / 16384.0f;
		q->z = (float) qI[3] / 16384.0f;
		return 0;
	}
	return status; // int16 return value, indicates error if this line is reached
}
// uint8_t dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t dmpGetGyro32bit(int32_t *data, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (((uint32_t) packet[22] << 8) | packet[23]);
	data[1] = (((uint32_t) packet[24] << 8) | packet[25]);
	data[2] = (((uint32_t) packet[26] << 8) | packet[27]);
	return 0;
}
uint8_t dmpGetGyro16bit(int16_t *data, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (packet[22] << 8) | packet[23];
	data[1] = (packet[24] << 8) | packet[25];
	data[2] = (packet[26] << 8) | packet[27];
	return 0;
}
uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t *packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	v->x = (packet[22] << 8) | packet[23];
	v->y = (packet[24] << 8) | packet[25];
	v->z = (packet[26] << 8) | packet[27];
	return 0;
}
// uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw,
		VectorFloat *gravity)
{
	// get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is 2g)
	v->x = vRaw->x - gravity->x * 16384;
	v->y = vRaw->y - gravity->y * 16384;
	v->z = vRaw->z - gravity->z * 16384;
	return 0;
}

// uint8_t dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t dmpGetGravity(long *data, const uint8_t* packet);
uint8_t dmpGetGravity16bit(int16_t *data, const uint8_t *packet)
{
	/* +1g corresponds to +16384, sensitivity is 2g. */
	int16_t qI[4];
	uint8_t status = dmpGetQuaternion16bit(qI, packet);
	data[0] = ((int32_t) qI[1] * qI[3] - (int32_t) qI[0] * qI[2]) / 16384;
	data[1] = ((int32_t) qI[0] * qI[1] + (int32_t) qI[2] * qI[3]) / 16384;
	data[2] = ((int32_t) qI[0] * qI[0] - (int32_t) qI[1] * qI[1]
			- (int32_t) qI[2] * qI[2] + (int32_t) qI[3] * qI[3])
			/ (int32_t) (2 * 16384L);
	return status;
}

uint8_t dmpGetGravityVectQuat(VectorFloat *v, Quaternion *q)
{
	v->x = 2 * (q->x * q->z - q->w * q->y);
	v->y = 2 * (q->w * q->x + q->y * q->z);
	v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
	return 0;
}
// uint8_t dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t dmpGetEIS(long *data, const uint8_t* packet);

uint8_t dmpGetEuler(float *data, Quaternion *q)
{
	data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z,
			2 * q->w * q->w + 2 * q->x * q->x - 1) * 57.29;    // psi
	data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y) * 57.29;         // theta
	data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x,
			2 * q->w * q->w + 2 * q->z * q->z - 1) * 57.29;    // phi
	return 0;
}

#ifdef USE_OLD_DMPGETYAWPITCHROLL
uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
  // yaw: (about Z axis)
  data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
  // pitch: (nose up/down, about Y axis)
  data[1] = atan(gravity->x / sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
  // roll: (tilt left/right, about X axis)
  data[2] = atan(gravity->y / sqrt(gravity->x * gravity->x + gravity->z * gravity->z));
  return 0;
}
#else
uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
	// yaw: (about Z axis)
	data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z,
			2 * q->w * q->w + 2 * q->x * q->x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan2(gravity->x,
			sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan2(gravity->y, gravity->z);
	if (gravity->z < 0)
	{
		if (data[1] > 0)
		{
			data[1] = PI - data[1];
		}
		else
		{
			data[1] = -PI - data[1];
		}
	}
	return 0;
}
#endif

// uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData)
{
	(void) dmpData;    // unused parameter
	/*for (uint8_t k = 0; k < dmpPacketSize; k++) {
	 if (dmpData[k] < 0x10) Serial.print("0");
	 Serial.print(dmpData[k], HEX);
	 Serial.print(" ");
	 }
	 Serial.print("\n");*/
	// Serial.println((uint16_t)dmpPacketBuffer);
	return 0;
}
uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed)
{
	uint8_t status;
	uint8_t buf[dmpPacketSize];
	for (uint8_t i = 0; i < numPackets; i++)
	{
		// read packet from FIFO
		getFIFOBytes(buf, dmpPacketSize);

		// process packet
		if ((status = dmpProcessFIFOPacket(buf)) > 0)
			return status;

		// increment external process count variable, if supplied
		if (processed != 0)
			(*processed)++;
	}
	return 0;
}

// uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t dmpInitFIFOParam();
// uint8_t dmpCloseFIFO();
// uint8_t dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t dmpDecodeQuantizedAccel();
// uint32_t dmpGetGyroSumOfSquare();
// uint32_t dmpGetAccelSumOfSquare();
// void dmpOverrideQuaternion(long *q);
uint16_t dmpGetFIFOPacketSize(void)
{
	return dmpPacketSize;
}

uint8_t dmpGetCurrentFIFOPacket(uint8_t *data)
{    // overflow proof
	return (GetCurrentFIFOPacket(data, 28));
}
