#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
using namespace std;

//UTCʱ��ṹ��
struct Time
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
};

//��˹���������ṹ��
struct GaussParameter
{
	float A;
	float b;
	float sigma;
};


bool isHeaderRight(uint8_t header[8]);

//��˹ƽ����
void gau_kernel(float kernel[], int size, float sigma);

//��˹ƽ������
void gaussian(float src[], float dst[]);

//����������
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


