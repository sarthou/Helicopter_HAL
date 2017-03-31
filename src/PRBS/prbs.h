/*
 * prbs.h
 *
 *  Created on: 22 mars 2017
 *      Author: Julia
 */
/* Génération d'une séquence SBPA de taille 2^16 (N=16)
 *
 */
#ifndef PRBS_PRBS_H_
#define PRBS_PRBS_H_

#include <stdint.h>

// registres à comparer : 4-13-15-16


class PRBS
{
public:
	PRBS(uint16_t min, uint16_t max, uint16_t seed);

	// renvoie la valeur suivante
	uint16_t nextValue();

private:
uint16_t m_reg;
uint16_t m_max;
uint16_t m_min;

uint16_t getMem(int index);

void setMem(int index, uint16_t value);

};


#endif /* PRBS_PRBS_H_ */
