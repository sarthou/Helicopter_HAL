/*
 * CommunicationProtocol.h
 *
 *  Created on: 22 févr. 2017
 *      Author: JulienCombattelli
 */

#ifndef COMMUNICATIONPROTOCOL_H_
#define COMMUNICATIONPROTOCOL_H_

#include <stdint.h>
#include <cstddef>

/*
 * TODO:
 * use mbed::Serial asynchronous I/O functions to avoid blocking code
 */

enum FrameType
{
	FrameType_Initialization = 0x11,
	FrameType_SignalRotorMain = 0x12,
	FrameType_SignalRotorTail = 0x13,
	FrameType_Start = 0x10,
	FrameType_Stop = 0xFF
};

void receive(uint8_t buffer[], size_t nbytes);

inline uint16_t to_uint16(uint8_t buffer[]) { return buffer[1] << 8 | buffer[0]; }

inline uint32_t to_uint32(uint8_t buffer[]) { return buffer[3] << 24 | buffer[2] << 16 | buffer[1] << 8 | buffer[0]; }


#endif /* COMMUNICATIONPROTOCOL_H_ */
