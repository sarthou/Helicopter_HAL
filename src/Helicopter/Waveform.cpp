/*
 * Waveform.cpp
 *
 *  Created on: 22 févr. 2017
 *      Author: JulienCombattelli
 */

#include "Waveform.h"

StepWaveform::StepWaveform(uint32_t tstart, uint32_t finalValue) :
	m_tstart(tstart), m_finalValue(finalValue)
{

}

uint32_t StepWaveform::generate(uint32_t currentTime)
{
	if(currentTime > m_tstart)
		return m_finalValue;
	else
		return 0;
}


RampWaveform::RampWaveform(uint32_t tstart, uint32_t slopeFactor) :
	m_tstart(tstart), m_slopeFactor(slopeFactor)
{

}

uint32_t RampWaveform::generate(uint32_t currentTime)
{
	if(currentTime > m_tstart)
		return uint32_t(m_slopeFactor)*currentTime;
	else
		return 0;
}
