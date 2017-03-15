/*
 * CommunicationProtocol.cpp
 *
 *  Created on: 22 févr. 2017
 *      Author: JulienCombattelli
 */

#include "CommunicationProtocol.h"
#include <cstdio>

void receive(uint8_t buffer[], size_t nbytes)
{
	for(size_t i = 0 ; i < nbytes ; i++)
		buffer[i] = uint8_t(getchar());
}
