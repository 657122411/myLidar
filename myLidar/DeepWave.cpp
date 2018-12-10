#include "DeepWave.h"

#define TimeDifference 8	//��UTC��ʱ��

DeepWave::DeepWave()
{
}

DeepWave::~DeepWave()
{
}

void DeepWave::GetDeepData(HS_Lidar & hs)
{
	//GPS->UTC->BeiJing
	PGPSTIME pgt = new GPSTIME;
	PCOMMONTIME pct = new COMMONTIME;
	pgt->wn = (int)hs.header.nGPSWeek;
	pgt->tow.sn = (long)hs.header.dGPSSecond;
	pgt->tow.tos = 0;
	GPSTimeToCommonTime(pgt, pct);
	m_time.year = pct->year;
	m_time.month = pct->month;
	m_time.day = pct->day;
	m_time.hour = pct->hour + TimeDifference;	//ֱ��ת��Ϊ����ʱ��
	m_time.minute = pct->minute;
	m_time.second = pct->second;
	delete pgt;
	delete pct;

	//ȡ����ͨ����ˮ����
	vector<int >::iterator it;//����������
	for (it = hs.deepData2.begin(); it != hs.deepData2.end(); ++it) 
	{
		m_BlueDeep.push_back((float)*it);
	}
	for (it = hs.deepData3.begin(); it != hs.deepData3.end(); ++it) 
	{
		m_GreenDeep.push_back((float)*it);
	}
}
