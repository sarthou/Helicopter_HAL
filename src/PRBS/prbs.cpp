#include "prbs.h"
#include "UART/Uart_driver.h"

#define BO 1
#define NO 0
#define ACTION 1
#define NVALSBPA  511

Sequence::Sequence()
{
	m_reg[8]=1;
	for(unsigned int i=7;i>0;i--)
	{
		m_reg[i]=0;
	}
}

bool Sequence::nextValue()
{
	bool val=0;
	if (m_reg[4]!=m_reg[8])
	{
		val=1;
	}
	for (unsigned int i=8;i>0;i--)
	{
		m_reg[i]=m_reg[i-1];
	}
	m_reg[0]=val;
	return val;
}


