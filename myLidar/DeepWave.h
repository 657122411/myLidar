#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
#include "levmar.h"
using namespace std;


//��˹���������ṹ��
struct DeepGaussParameter
{
	float DA;	//�����Ymax��
	float Db;	//�������(�Գ���)
	float Dsigma;//�����ȣ������
	bool deepwavetype;
};


//�������ݵı�׼��
float calculateDeepSigma(vector<float> resultSet);

//��ˮ����������
class DeepWave
{
public:
	DeepWave();
	~DeepWave();
	void GetDeepData(HS_Lidar &hs);					//��ȡ��ˮ��������
	void DeepFilter(vector<float> &srcWave, float &noise);						//�˲�ƽ��
	void DeepResolve(vector<float> &srcWave, vector<DeepGaussParameter> &waveParam, float &noise);	//�ֽ��˹��������
	void DeepOptimize(vector<float> &srcWave, vector<DeepGaussParameter> &waveParam);//�����Ż���LM��

	static bool ostreamFlag;												//�������������Ȥͨ������
	friend ostream &operator<<(ostream &stream, const DeepWave &deepwave);	//�Զ��������Ϣ
	Time m_time;									//UTCʱ��

	vector<float> m_BlueDeep;						//CH2ͨ����ˮ����
	vector<float> m_GreenDeep;						//CH3ͨ����ˮ����
	float m_BlueDeepNoise;								//CH2ͨ�����������
	float m_GreenDeepNoise;								//CH3ͨ�����������
	vector<DeepGaussParameter> m_BlueDeepGauPra;			//CH2���ݸ�˹��������
	vector<DeepGaussParameter> m_GreenDeepGauPra;			//CH3���ݸ�˹��������
	vector<DeepGaussParameter>::iterator DeepgaussPraIter;	//��˹�����ṹ�������

	float blueDeepDepth;								//CH2ͨ���ļ���ˮ��
	float greenDeepDepth;								//CH3ͨ���ļ���ˮ��
	void calculateDeepDepth(vector<DeepGaussParameter> &waveParam, float &BorGDepth);	//���ݻز����ݼ���ˮ��
};