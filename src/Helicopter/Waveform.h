/*
 * Waveform.h
 *
 *  Created on: 22 févr. 2017
 *      Author: JulienCombattelli
 */

#ifndef WAVEFORM_H_
#define WAVEFORM_H_

#include <stdint.h>
#include <PRBS/prbs.h>

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

class PRBSWaveform : public Waveform
{
public:
	PRBSWaveform(uint32_t tstart,uint32_t min, uint32_t max, uint16_t seed);
	virtual uint32_t generate(uint32_t currentTime);

private:
	uint32_t m_tstart;
	Prbs m_prbs;
};

#endif /* WAVEFORM_H_ */
