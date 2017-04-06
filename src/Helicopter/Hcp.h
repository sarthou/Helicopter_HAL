/*
 * Hcp.h
 *
 *  Created on: 22 févr. 2017
 *      Author: JulienCombattelli
 */

#ifndef HCP_H_
#define HCP_H_

#include <stdint.h>
#include <stddef.h>

enum HCP_FrameType
{
	FrameType_ManualRotorMain 	= 0x01,
	FrameType_ManualRotorTail 	= 0x02,
	FrameType_Start 			= 0x10,
	FrameType_Initialization 	= 0x11,
	FrameType_SignalRotorMain 	= 0x12,
	FrameType_SignalRotorTail 	= 0x13,
	FrameType_SignalSensors		= 0x14,
	FrameType_StartTransmission = 0x1E,
	FrameType_Stop 				= 0xFF
};

inline uint16_t HCP_toUint16(uint8_t buffer[]) { return buffer[1] << 8 | buffer[0]; }

inline uint32_t HCP_toUint32(uint8_t buffer[]) { return buffer[3] << 24 | buffer[2] << 16 | buffer[1] << 8 | buffer[0]; }

#endif /* HCP_H_ */
