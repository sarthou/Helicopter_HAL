#include <Prbs/Prbs.h>

Prbs::Prbs(uint16_t min,uint16_t max,uint16_t seed)
{
	m_reg=seed;
	m_max=max;
	m_min=min;
}

uint16_t Prbs::nextValue()
{
	uint16_t val,val4,val13,val15,val16;
	val4=getMem(4);
	val13=getMem(13);
	val15=getMem(15);
	val16=getMem(16);

	val=val4^val13^val15^val16;

	setMem(16,val);

	return ((m_max-m_min)*val+m_min);

	/*if (val==1)
	{
		val=m_max;
	}
	else
	{
		val=m_min;
	}
	return val;
	*/
}

uint16_t Prbs::getMem(int index)
{
	return ((m_reg>>(16-index))&0x1);
}

void Prbs::setMem(int index, uint16_t value)
{
	m_reg=(m_reg>>1)|(value<<(index-1));
}


