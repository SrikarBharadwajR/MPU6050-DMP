#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

// #define I2CDEV_SERIAL_DEBUG
#define I2CDEVLIB_WIRE_BUFFER_LENGTH 32
#define I2CDEV_DEFAULT_READ_TIMEOUT 100    // 1000ms default read timeout (modify with "I2Cdev::readTimeout = [ms];")

void I2Cdev_init(I2C_HandleTypeDef *hi2c);
int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data,
		uint16_t timeout, void *wireObj);
int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
		uint16_t *data, uint16_t timeout, void *wireObj);
int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj);
int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj);
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data,
		uint16_t timeout, void *wireObj);
int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data,
		uint16_t timeout, void *wireObj);
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
		uint8_t *data, uint16_t timeout, void *wireObj);
int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
		uint16_t *data, uint16_t timeout, void *wireObj);

bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data,
		void *wireObj);
bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data,
		void *wireObj);
bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint8_t data, void *wireObj);
bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint16_t data, void *wireObj);
bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, void *wireObj);
bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data, void *wireObj);
bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data,
		void *wireObj);
bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
		uint16_t *data, void *wireObj);

#endif /* _I2CDEV_H_ */
