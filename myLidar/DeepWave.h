#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
#include "levmar.h"
using namespace std;


//计算数据的标准差
float calculateDeepSigma(vector<float> resultSet);

//深水波形数据类
class DeepWave
{
public:
	DeepWave();
	~DeepWave();
	void GetDeepData(HS_Lidar &hs);											//获取深水区域数据
	void DeepFilter(vector<float> &srcWave, float &noise);					//滤波平滑
	void DeepResolve(vector<float> &srcWave, vector<float> &waveParam, float &noise);	//分解分量索引
	void DeepOptimize(vector<float> &srcWave, vector<float> &waveParam);	//迭代优化（LM）

	static bool ostreamFlag;												//控制流输出的兴趣通道数据
	friend ostream &operator<<(ostream &stream, const DeepWave &deepwave);	//自定义输出信息
	Time m_time;									//UTC时间

	vector<float> m_BlueDeep;						//CH2通道深水数据
	vector<float> m_GreenDeep;						//CH3通道深水数据
	float m_BlueDeepNoise;							//CH2通道的随机噪声
	float m_GreenDeepNoise;							//CH3通道的随机噪声
	vector<float> m_BlueDeepPra;					//CH2数据峰值点索引
	vector<float> m_GreenDeepPra;					//CH3数据峰值点索引
	vector<float>::iterator DeepPraIter;			//峰值点索引迭代器

	float blueDeepDepth;							//CH2通道的计算水深
	float greenDeepDepth;							//CH3通道的计算水深
	void calculateDeepDepth(vector<float> &waveParam, float &BorGDepth);	//根据回波数据计算水深
};