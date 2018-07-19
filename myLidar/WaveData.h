#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
using namespace std;

//UTC时间结构体
struct Time
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
};

//高斯函数参数结构体
struct GaussParameter
{
	float A;
	float b;
	float sigma;
};


bool isHeaderRight(uint8_t header[8]);

//高斯平滑核
void gau_kernel(float kernel[], int size, float sigma);

//高斯平滑函数
void gaussian(float src[], float dst[]);

//波形数据类
class WaveData
{
public:
	WaveData();
	~WaveData();
	void GetData(HS_Lidar &hs);
	void Filter(vector<float> &srcWave);
	void Resolve(vector<float> &srcWave,vector<GaussParameter> &waveParam);
	void Optimize(vector<float> &srcWave,vector<GaussParameter> &waveParam);


	Time m_time;
	vector<float> m_BlueWave;
	vector<float> m_GreenWave;
	vector<GaussParameter> m_BlueGauPra;
	vector<GaussParameter> m_GreenGauPra;
	vector<GaussParameter>::iterator gaussPraIter;
};


