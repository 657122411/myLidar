#include "WaveData.h"


//判断帧头是否正确
bool isHeaderRight(uint8_t header[8])
{
	uint8_t headerSign[] = { 1, 35, 69, 103, 137, 171, 205, 239 };
	bool returnVal = true;
	for (size_t i = 0; i < 8; i++)
	{
		if (header[i] != headerSign[i])
		{
			returnVal = false;
			break;
		}
	}
	return returnVal;
}




/*功能：  高斯核生成
//kernel：存储生成的高斯核
//size：  核的大小
//sigma： 正态分布标准差
*/
void gau_kernel(float kernel[], int size, float sigma)
{
	if (size <= 0 || sigma == 0)
		return;
	int x;
	float sum = 0;
	int m = (size - 1) / 2;

	//get kernel	
	for (x = 0; x <size; x++)
	{
		kernel[x] = (1 / sigma * sqrt(2 * 3.1415)) * exp(-(x - m)*(x - m) / 2 * sigma*sigma);
		sum += kernel[x];
	}

	//normal
	for (x = 0; x < size; x++)
	{
		kernel[x] /= sum;
	}
}


/*功能： 高斯模糊
//src：  输入原图
//dst：  模糊图像
//size： 核的大小
//sigma：正态分布标准差
*/
void gaussian(float src[], float dst[])
{
	float kernel[5];
	gau_kernel(kernel, 5, 1);
	//gaussian卷积,此时边界没加处理
	for (int i = (5 - 1) / 2; i <= 319 - (5 - 1) / 2; i++)
	{
		dst[i] = src[i - 2] * kernel[0] + src[i - 1] * kernel[1] + src[i] * kernel[2] + src[i + 1] * kernel[3] + src[i + 2] * kernel[4];
	}
}


WaveData::WaveData()
{

};

WaveData::~WaveData()
{

};


/*提取原始数据中的兴趣区域数据*/
void WaveData::GetData(HS_Lidar &hs)
{
	PGPSTIME pgt = new GPSTIME;
	PCOMMONTIME pct = new COMMONTIME;
	pgt->wn = (int)hs.header.nGPSWeek;
	pgt->tow.sn = (long)hs.header.dGPSSecond;
	pgt->tow.tos = 0;
	GPSTimeToCommonTime(pgt, pct);
	m_time.year = pct->year;
	m_time.month = pct->month;
	m_time.day = pct->day;
	m_time.hour = pct->hour;
	m_time.minute = pct->minute;
	m_time.minute = pct->second;
	delete pgt;
	delete pct;

	m_BlueWave.assign(&hs.CH2.nD0[0], &hs.CH2.nD0[320]);
	m_GreenWave.assign(&hs.CH3.nD0[0], &hs.CH3.nD0[320]);

};


/*去噪滤波函数*/
void WaveData::Filter(vector<float> &srcWave)
{
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	gaussian(&srcWave[0], &dstWave[0]);
	srcWave.assign(dstWave.begin(), dstWave.end());
	dstWave.clear();
};


/*高斯分量分解函数*/
void WaveData::Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam)
{
	float data[320],temp[320];
	int i = 0, m = 0;
	for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
	{
		data[i] = *iter;
	}

	float min = data[0];//噪声最小值
	for (m = 0; m < 320; m++)
	{
		temp[m] = data[m];
		if (data[m] < min)	
			min = data[m];
	}

	//去噪
	for (m = 0; m < 320; m++)
	{
		temp[m] -= min;
	}

	srcWave.assign(&temp[0],&temp[320]);

	//高斯分量
	float A;//振幅
	float b, tg, tgl, tgr;//脉冲距离，峰值时间位置，半峰时间位置（左右）


	do
	{
		A = 0;
		//找最大值并记录位置
		for (m = 0; m < 320; m++)
		{
			if (temp[m] > A)
			{
				A = temp[m];
				b = m;
			}
		}

		//寻找半宽位置
		for (m = b; m < 319; m++)
		{
			if ((temp[m - 1] > A / 2) && (temp[m + 1] < A / 2))
			{
				tgr = m;
				break;
			}
		}
		for (m = b; m > 0; m--)
		{
			if ((temp[m - 1] < A / 2) && (temp[m + 1] > A / 2))
			{
				tgl = m;
				break;
			}
		}
		if ((b - tgl) > (tgr - b))
		{
			tg = tgr;
		}
		else
		{
			tg = tgl;
		}


		//计算sigma
		float sigma = fabs(tg - b) / sqrt(2 * log(2));

		GaussParameter param{A,b,sigma};
		waveParam.push_back(param);

		//剥离
		for (m = 0; m < 320; m++)
		{
			if (temp[m] > A*exp(-(m - b)*(m - b) / (2 * sigma*sigma)))
			{
				temp[m] -= A*exp(-(m - b)*(m - b) / (2 * sigma*sigma));
			}
			else
				temp[m] = 0;
		}

		//判断是否继续剥离
		A = 0;
		for (m = 0; m < 320; m++)
		{
			if (temp[m] > A)
			{
				A = temp[m];
			}
		}

		//获取向量中所存结构体的第一个波峰值作阈值参考量
		gaussPraIter = waveParam.begin();
	} while (A >= 1.5*20/*Gnoise*/);//循环条件!!!值得探讨





	//对高斯分量做筛选（时间间隔小于一定值的剔除能量较小的分量）
	//先将差值过小vector对象的sigma值设为0
	for (int i = 0; i<waveParam.size() - 1; i++)
	{
		for (int j = i + 1; j < waveParam.size(); j++)
		{
			if (abs(waveParam.at(i).b - waveParam.at(j).b) < 5)
			{
				if (waveParam.at(i).A >= waveParam.at(j).A)
				{
					waveParam.at(j).sigma = 0;
				}
				else
				{
					waveParam.at(i).sigma = 0;
				}
			}
		}
	}

	//再将sigma为0值的分量删除
	for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end();)
	{
		if (gaussPraIter->sigma == 0)
		{
			gaussPraIter = waveParam.erase(gaussPraIter);
		}
		else
		{
			++gaussPraIter;
		}
	}


};


void WaveData::Optimize(vector<float> &originWave,vector<GaussParameter> &waveParam)
{

};



