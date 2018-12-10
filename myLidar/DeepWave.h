#pragma once
#include <iostream>
#include <vector>
#include "HS_Lidar.h"
#include "TimeConvert.h"
using namespace std;


//UTCʱ��ṹ��
//struct Time
//{
//	int year;
//	int month;
//	int day;
//	int hour;
//	int minute;
//	int second;
//};

class DeepWave
{
public:
	DeepWave();
	~DeepWave();
	void GetDeepData(HS_Lidar &hs);					//��ȡ��ˮ��������

	Time m_time;									//UTCʱ��

	vector<float> m_BlueDeep;						//CH2ͨ����ˮ����
	vector<float> m_GreenDeep;						//CH3ͨ����ˮ����

};