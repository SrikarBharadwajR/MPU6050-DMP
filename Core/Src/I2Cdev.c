#include "I2Cdev.h"

static I2C_HandleTypeDef *I2Cdev_hi2c;

const uint16_t readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

/** Sets device handle to use for communications
 * You can call this function and set any other device at any moment
 */
void I2Cdev_init(I2C_HandleTypeDef *hi2c)
{
	I2Cdev_hi2c = hi2c;
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data,
		uint16_t timeout, void *wireObj)
{
	uint8_t b;
	uint8_t count = readByte(devAddr, regAddr, &b, timeout, wireObj);
	*data = b & (1 << bitNum);
	return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
		uint16_t *data, uint16_t timeout, void *wireObj)
{
	uint16_t b;
	uint8_t count = readWord(devAddr, regAddr, &b, timeout, wireObj);
	*data = b & (1 << bitNum);
	return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t count, b;
	if ((count = readByte(devAddr, regAddr, &b, timeout, wireObj)) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj)
{
	// 1101011001101001 read byte
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	//    010           masked
	//           -> 010 shifted
	uint8_t count;
	uint16_t w;
	if ((count = readWord(devAddr, regAddr, &w, timeout, wireObj)) != 0)
	{
		uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		w &= mask;
		w >>= (bitStart - length + 1);
		*data = w;
	}
	return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data,
		uint16_t timeout, void *wireObj)
{
	return readBytes(devAddr, regAddr, 1, data, timeout, wireObj);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data,
		uint16_t timeout, void *wireObj)
{
	return readWords(devAddr, regAddr, 1, data, timeout, wireObj);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
		uint8_t *data, uint16_t timeout, void *wireObj)
{
	uint16_t tout = timeout > 0 ? timeout : I2CDEV_DEFAULT_READ_TIMEOUT;

	HAL_I2C_Master_Transmit(I2Cdev_hi2c, devAddr << 1, &regAddr, 1, tout);
	if (HAL_I2C_Master_Receive(I2Cdev_hi2c, devAddr << 1, data, length, tout)
			== HAL_OK)
		return length;
	return -1;
}
/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
		uint16_t *data, uint16_t timeout, void *wireObj)
{
	uint16_t tout = timeout > 0 ? timeout : I2CDEV_DEFAULT_READ_TIMEOUT;
	uint8_t buffer[length * 2];  // Temporary buffer to hold raw bytes

	// Send register address
	if (HAL_I2C_Master_Transmit(I2Cdev_hi2c, devAddr << 1, &regAddr, 1, tout)
			!= HAL_OK)
	{
		return -1;  // Transmission failed
	}

	// Read raw data into buffer
	if (HAL_I2C_Master_Receive(I2Cdev_hi2c, devAddr << 1, buffer, length * 2,
			tout) != HAL_OK)
	{
		return -1;  // Reception failed
	}

	// Convert to big-endian: combine high byte and low byte into 16-bit words
	for (uint8_t i = 0; i < length; i++)
	{
		data[i] = ((uint16_t) buffer[2 * i] << 8) | buffer[2 * i + 1]; // High byte first
	}

	return length;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data,
		void *wireObj)
{
	uint8_t b;
	readByte(devAddr, regAddr, &b, readTimeout, wireObj);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return writeByte(devAddr, regAddr, b, wireObj);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data,
		void *wireObj)
{
	uint16_t w;
	readWord(devAddr, regAddr, &w, readTimeout, wireObj);
	w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
	return writeWord(devAddr, regAddr, w, wireObj);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint8_t data, void *wireObj)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	if (readByte(devAddr, regAddr, &b, readTimeout, wireObj) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask;					  // zero all non-important bits in data
		b &= ~(mask);				// zero all important bits in existing byte
		b |= data;						  // combine data with existing byte
		return writeByte(devAddr, regAddr, b, wireObj);
	}
	else
	{
		return false;
	}
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint16_t data, void *wireObj)
{
	//              010 value to write
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	// 0001110000000000 mask word
	// 1010111110010110 original value (sample)
	// 1010001110010110 original & ~mask
	// 1010101110010110 masked | value
	uint16_t w;
	if (readWord(devAddr, regAddr, &w, readTimeout, wireObj) != 0)
	{
		uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask;					  // zero all non-important bits in data
		w &= ~(mask);				// zero all important bits in existing word
		w |= data;						  // combine data with existing word
		return writeWord(devAddr, regAddr, w, wireObj);
	}
	else
	{
		return false;
	}
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, void *wireObj)
{
	return writeBytes(devAddr, regAddr, 1, &data, wireObj);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data, void *wireObj)
{
	return writeWords(devAddr, regAddr, 1, &data, wireObj);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data,
		void *wireObj)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2Cdev_hi2c, devAddr << 1,
			regAddr, I2C_MEMADD_SIZE_8BIT, data, length, 1000);
	return status == HAL_OK;
}
/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
//bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
//		uint16_t *data, void *wireObj)
//{
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2Cdev_hi2c, devAddr << 1,
//			regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) data,
//			sizeof(uint16_t) * length, 1000);
//	return status == HAL_OK;
//}
bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
		uint16_t *data, void *wireObj)
{
	uint8_t tempData[length * 2];  // Temporary buffer for byte-swapped data
	for (uint8_t i = 0; i < length; i++)
	{
		tempData[2 * i] = (data[i] >> 8) & 0xFF;    // High byte first
		tempData[2 * i + 1] = data[i] & 0xFF;       // Low byte second
	}

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2Cdev_hi2c, devAddr << 1,
			regAddr, I2C_MEMADD_SIZE_8BIT, tempData, length * 2, 1000);
	return status == HAL_OK;
}
