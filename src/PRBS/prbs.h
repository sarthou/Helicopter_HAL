/*
 * prbs.h
 *
 *  Created on: 22 mars 2017
 *      Author: Julia
 */
/* Génération d'une séquence SBPA de taille 255 (N=9)
 *
 */

#ifndef PRBS_PRBS_H_
#define PRBS_PRBS_H_

#define NB 9

class Sequence
{
public:
	Sequence();

	bool nextValue();

private:
bool m_reg[NB];
};



#endif /* PRBS_PRBS_H_ */
