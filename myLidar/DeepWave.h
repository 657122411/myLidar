#pragma once
#include <iostream>
#include <vector>
#include "HS_Lidar.h"
#include "TimeConvert.h"
using namespace std;


//UTC时间结构体
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
	void GetDeepData(HS_Lidar &hs);					//获取深水区域数据

	Time m_time;									//UTC时间

	vector<float> m_BlueDeep;						//CH2通道深水数据
	vector<float> m_GreenDeep;						//CH3通道深水数据

};