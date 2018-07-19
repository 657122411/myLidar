#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
#include "levmar.h"
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
	float A;	//�����Ymax��
	float b;	//�������(�Գ���)
	float sigma;//�����ȣ������
};


//����������
class WaveData
{
public:
	WaveData();
	~WaveData();
	void GetData(HS_Lidar &hs);												//��ȡ��Ȥ����
	void Filter(vector<float> &srcWave);									//�˲�ƽ��
	void Resolve(vector<float> &srcWave,vector<GaussParameter> &waveParam);	//�ֽ��˹��������
	void Optimize(vector<float> &srcWave,vector<GaussParameter> &waveParam);//�����Ż���LM��

	Time m_time;									//UTCʱ��
	vector<float> m_BlueWave;						//CH2ͨ������
	vector<float> m_GreenWave;						//CH3ͨ������
	vector<GaussParameter> m_BlueGauPra;			//CH2���ݸ�˹��������
	vector<GaussParameter> m_GreenGauPra;			//CH3���ݸ�˹��������
	vector<GaussParameter>::iterator gaussPraIter;	//��˹�����ṹ�������
};


