/*
 * Waveform.h
 *
 *  Created on: 22 févr. 2017
 *      Author: JulienCombattelli
 */

#ifndef WAVEFORM_H_
#define WAVEFORM_H_

#include <stdint.h>

enum WaveformType
{
	WaveformType_Step = 0x00,
	WaveformType_Ramp = 0x01,
	WaveformType_PRBS = 0x02
};

class Waveform
{
public:
	virtual uint32_t generate(uint32_t currentTime) = 0;
	virtual ~Waveform() {};
};

class StepWaveform : public Waveform
{
public:
	StepWaveform(uint32_t tstart, uint32_t finalValue);
	virtual uint32_t generate(uint32_t currentTime);

private:
	uint32_t m_tstart;
	uint32_t m_finalValue;
};

class RampWaveform : public Waveform
{
public:
	RampWaveform(uint32_t tstart, uint32_t slopeFactor);
	virtual uint32_t generate(uint32_t currentTime);

private:
	uint32_t m_tstart;
	uint32_t m_slopeFactor;
};

#endif /* WAVEFORM_H_ */
