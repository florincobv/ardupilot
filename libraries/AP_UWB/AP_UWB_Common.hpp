/*
 * Common.hpp
 *
 *  Created on: 28 jun. 2022
 *      Author: walrh
 */

#pragma once

#include <stdint.h>

/**
 * Union used for translating option values between long and float (some options need long values, others need float values)
 * Union can also be used to construct and deconstruct data packets.
 */


#pragma pack(push)
#pragma pack(1)


typedef union longOrFloatU {
	unsigned long longVal;
	float floatVal;
	uint8_t packetVal[4];
} longOrFloat;

#pragma pack(pop)

uint16_t bufferToHWord(uint8_t *data);
uint32_t bufferToWord(uint8_t *data);
uint64_t bufferToDWord(uint8_t *data);
double bufferToDouble(uint8_t *data);
float bufferToFloat(uint8_t *data);

void hWordToBuffer(uint8_t *data, uint16_t value);
void wordToBuffer(uint8_t *data, uint32_t value);
void dWordToBuffer(uint8_t *data, uint64_t value);
void floatToBuffer(uint8_t *data, float value);
void doubleToBuffer(uint8_t *data, double value);
void floatToBuffer(uint8_t *data, float value);

