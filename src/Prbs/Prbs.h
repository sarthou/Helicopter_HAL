/*
 * Prbs.h
 *
 *  Created on: 22 mars 2017
 *      Author: Julia
 *
 * Pseudo random binary sequence generator with a size of 2^16 (N=16)
 */
#ifndef PRBS_PRBS_H_
#define PRBS_PRBS_H_

#include <stdint.h>

// register to compare : 4-13-15-16


class Prbs
{
public:
	Prbs(uint16_t min, uint16_t max, uint16_t seed);

	// return the next value of the sequence
	uint16_t nextValue();

private:
	uint16_t m_reg;
	uint16_t m_max;
	uint16_t m_min;

	uint16_t getMem(int index);

	void setMem(int index, uint16_t value);
};


#endif /* PRBS_PRBS_H_ */
