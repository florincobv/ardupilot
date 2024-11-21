/*
 * Common.cpp
 *
 *  Created on: 28 jun. 2022
 *      Author: walrh
 */

#include <cstring>

#include "AP_UWB_Common.hpp"

typedef union {
	uint8_t buffer[4];
	uint32_t value;
} WordBuffer_t;

typedef union {
	uint8_t buffer[8];
	uint64_t value;
} DWordBuffer_t;

typedef union {
	uint8_t buffer[8];
	double value;
} DoubleBuffer_t;

typedef union {
	uint8_t buffer[4];
	float value;
} FloatBuffer_t;

typedef union {
	uint8_t buffer[2];
	uint16_t value;
} HWordBuffer_t;


/* Function:	Convert the buffer array to an uint32_t
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 *
 * Param Out:	-
 *
 * Return:		- (uint32_t): Converted value
 */
uint16_t bufferToHWord(uint8_t *data)
{
	HWordBuffer_t buffer;
	memcpy(buffer.buffer, data, sizeof(buffer.buffer));
	return buffer.value;
}

/* Function:	Convert the buffer array to an uint32_t
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 *
 * Param Out:	-
 *
 * Return:		- (uint32_t): Converted value
 */
uint32_t bufferToWord(uint8_t *data)
{
	WordBuffer_t buffer;
	memcpy(buffer.buffer, data, sizeof(buffer.buffer));
	return buffer.value;
}

/* Function:	Convert the buffer array to an uint32_t
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 *
 * Param Out:	-
 *
 * Return:		- (uint32_t): Converted value
 */
uint64_t bufferToDWord(uint8_t *data)
{
	DWordBuffer_t buffer;
	memcpy(buffer.buffer, data, sizeof(buffer.buffer));
	return buffer.value;
}

/* Function:	Convert the buffer array to an uint32_t
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 *
 * Param Out:	-
 *
 * Return:		- (uint32_t): Converted value
 */
double bufferToDouble(uint8_t *data)
{
	DoubleBuffer_t buffer;
	memcpy(buffer.buffer, data, sizeof(buffer.buffer));
	return buffer.value;
}

/**
 * Convert the buffer array to a float
 *
 * @param {uint8_t*} data - Pointer to buffer which need to be converted
 * @return {float} - Converted value
 */
float bufferToFloat(uint8_t *data)
{
	FloatBuffer_t buffer;
	memcpy(buffer.buffer, data, sizeof(buffer.buffer));
	return buffer.value;
}

/* Function:	Convert the uint32_t to a buffer array
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 * 				- (uint32_t) value:  Value to convert
 *
 * Param Out:	-
 *
 * Return:		-
 */
void wordToBuffer(uint8_t *data, uint32_t value)
{
	WordBuffer_t buffer;
	buffer.value = value;
	memcpy(data, buffer.buffer, sizeof(buffer.value));
}

/* Function:	Convert the uint16_t to a buffer array
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 * 				- (uint16_t) value:  Value to convert
 *
 * Param Out:	-
 *
 * Return:		-
 */
void hWordToBuffer(uint8_t *data, uint16_t value)
{
	HWordBuffer_t buffer;
	buffer.value = value;
	memcpy(data, buffer.buffer, sizeof(buffer.value));
}

/* Function:	Convert the uint64_t to a buffer array
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 * 				- (uint64_t) value:  Value to convert
 *
 * Param Out:	-
 *
 * Return:		-
 */
void dWordToBuffer(uint8_t *data, uint64_t value)
{
	DWordBuffer_t buffer;
	buffer.value = value;
	memcpy(data, buffer.buffer, sizeof(buffer.value));
}

/* Function:	Convert the double to a buffer array
 *
 * Param In:	- (uint8_t*) buffer: A pointer to the buffer which need to be converted
 * 				- (double) value:  Value to convert
 *
 * Param Out:	-
 *
 * Return:		-
 */
void doubleToBuffer(uint8_t *data, double value)
{
	DoubleBuffer_t buffer;
	buffer.value = value;
	memcpy(data, buffer.buffer, sizeof(buffer.value));
}

/*
 * Convert float to a buffer array
 *
 * @param {uint8_t*} data - Convert the double to a buffer array
 * @param {float} value - Value to convert
 */
void floatToBuffer(uint8_t *data, float value)
{
	FloatBuffer_t buffer;
	buffer.value = value;
	memcpy(data, buffer.buffer, sizeof(buffer.value));
}

