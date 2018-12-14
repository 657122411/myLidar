#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
#include "levmar.h"
using namespace std;


//高斯函数参数结构体
struct DeepGaussParameter
{
	float DA;	//振幅（Ymax）
	float Db;	//脉冲距离(对称轴)
	float Dsigma;//脉冲宽度（宽幅）
	bool deepwavetype;
};


//计算数据的标准差
float calculateDeepSigma(vector<float> resultSet);

//深水波形数据类
class DeepWave
{
public:
	DeepWave();
	~DeepWave();
	void GetDeepData(HS_Lidar &hs);					//获取深水区域数据
	void DeepFilter(vector<float> &srcWave, float &noise);						//滤波平滑
	void DeepResolve(vector<float> &srcWave, vector<DeepGaussParameter> &waveParam, float &noise);	//分解高斯分量参数
	void DeepOptimize(vector<float> &srcWave, vector<DeepGaussParameter> &waveParam);//迭代优化（LM）

	static bool ostreamFlag;												//控制流输出的兴趣通道数据
	friend ostream &operator<<(ostream &stream, const DeepWave &deepwave);	//自定义输出信息
	Time m_time;									//UTC时间

	vector<float> m_BlueDeep;						//CH2通道深水数据
	vector<float> m_GreenDeep;						//CH3通道深水数据
	float m_BlueDeepNoise;								//CH2通道的随机噪声
	float m_GreenDeepNoise;								//CH3通道的随机噪声
	vector<DeepGaussParameter> m_BlueDeepGauPra;			//CH2数据高斯分量参数
	vector<DeepGaussParameter> m_GreenDeepGauPra;			//CH3数据高斯分量参数
	vector<DeepGaussParameter>::iterator DeepgaussPraIter;	//高斯参数结构体迭代器

	float blueDeepDepth;								//CH2通道的计算水深
	float greenDeepDepth;								//CH3通道的计算水深
	void calculateDeepDepth(vector<DeepGaussParameter> &waveParam, float &BorGDepth);	//根据回波数据计算水深
};