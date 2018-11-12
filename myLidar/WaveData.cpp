#include "WaveData.h"

#define PulseWidth 4		//定义激光脉冲宽度做剥离阈值参考
#define TimeDifference 8	//与UTC的时差

#define BLUE true
#define GREEN false

#define c 0.3				//相对光速乘以纳秒
#define nwater 1.334				//水质的折射率

bool WaveData::ostreamFlag = BLUE;

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


/*功能:	 高斯模糊
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


/*功能：	假设两组高斯函数模型
//*p:	代求参数
//*x：  原始数据（测量值）
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void expfun2(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//写出参数与x[i]之间的关系式，由于这里方程的右边没有观测值，所以只有参数
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2])) 
			 + p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]));
	}

}


/*功能：	两组高斯函数模型的雅可比矩阵
//*p:	代求参数
//jac： 雅可比矩阵参数
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void jacexpfun2(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//写出雅克比矩阵
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
	}
}


/*功能：	假设三组高斯函数模型
//*p:	代求参数
//*x：  原始数据（测量值）
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void expfun3(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//写出参数与x[i]之间的关系式，由于这里方程的右边没有观测值，所以只有参数
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2])) 
			 + p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]))
			 + p[6] * exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*(2 * p[8]));
	}

}


/*功能：	三组高斯函数模型的雅可比矩阵
//*p:	代求参数
//jac： 雅可比矩阵参数
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void jacexpfun3(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//写出雅克比矩阵
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));

		jac[j++] = exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*p[8]);
		jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));
		jac[j++] = p[6] * (i - p[7])*(i - p[7]) / (p[8] * p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));
	}
}


/*功能：	假设四组高斯函数模型
//*p:	代求参数
//*x：  原始数据（测量值）
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void expfun4(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//写出参数与x[i]之间的关系式，由于这里方程的右边没有观测值，所以只有参数
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]))
			+ p[6] * exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*(2 * p[8]))
			+ p[9] * exp(-(i - p[10])*(i - p[10]) / (2 * p[11])*(2 * p[11]));
	}

}


/*功能：	四组高斯函数模型的雅可比矩阵
//*p:	代求参数
//jac： 雅可比矩阵参数
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void jacexpfun4(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//写出雅克比矩阵
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));

		jac[j++] = exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*p[8]);
		jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));
		jac[j++] = p[6] * (i - p[7])*(i - p[7]) / (p[8] * p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));

		jac[j++] = exp(-(i - p[10])*(i - p[10]) / (2 * p[11])*p[11]);
		jac[j++] = p[9] * (i - p[10]) / (p[11] * p[11])*exp(-(i - p[10])*(i - p[10]) / (2 * p[11] * p[11]));
		jac[j++] = p[9] * (i - p[10])*(i - p[10]) / (p[11] * p[11] * p[11])*exp(-(i - p[10])*(i - p[10]) / (2 * p[11] * p[11]));
	}
}


WaveData::WaveData()
{

};


WaveData::~WaveData()
{
	//手动释放vector内存，不知道有没有必要性
	vector<float>().swap(m_BlueWave);
	vector<float>().swap(m_GreenWave);
	vector<GaussParameter>().swap(m_BlueGauPra);
	vector<GaussParameter>().swap(m_GreenGauPra);
};


/*功能：	提取原始数据中的兴趣区域数据
//*&hs:	原始Lidar数据
*/
void WaveData::GetData(HS_Lidar &hs)
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
	m_time.hour = pct->hour + TimeDifference;	//直接转化为北京时间
	m_time.minute = pct->minute;
	m_time.second = pct->second;
	delete pgt;
	delete pct;

	//取蓝绿通道
	m_BlueWave.assign(&hs.CH2.nD0[0], &hs.CH2.nD0[320]);
	m_GreenWave.assign(&hs.CH3.nD0[0], &hs.CH3.nD0[320]);

};


/*功能：		预处理函数进行去噪滤波操作
//&srcWave:	通道原始数据
//&noise：	记录的噪声所属波段
*/
void WaveData::Filter(vector<float> &srcWave, float &noise)
{
	//高斯滤波去噪
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	gaussian(&srcWave[0], &dstWave[0]);

	noise = 0;
	//计算随机噪声:两次滤波前后的波形数据的峰值差的均方差（标准差）
	for (int i = 0; i < srcWave.size();i++)
	{
		noise += (srcWave.at(i) - dstWave.at(i)) * (srcWave.at(i) - dstWave.at(i));
	}
	noise = sqrt(noise / srcWave.size());

	srcWave.assign(dstWave.begin(), dstWave.end());
};


/*功能：			高斯分量分解函数
//&srcWave:		通道原始数据
//&waveParam：	该通道的高斯分量参数
*/
void WaveData::Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam, float &noise)
{
	//拷贝原始数据
	float data[320],temp[320];
	int i = 0, m = 0;
	for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
	{
		data[i] = *iter;
	}
	
	//将滤波后的数据最小值作为背景噪声
	float backgroundNoise = data[0];
	for (m = 0; m < 320; m++)
	{
		temp[m] = data[m];
		if (data[m] < backgroundNoise)
			backgroundNoise = data[m];
	}

	//所有数据除去环境噪声
	for (m = 0; m < 320; m++)
	{
		temp[m] -= backgroundNoise;
	}
	srcWave.assign(&temp[0],&temp[320]);

	float A;	//振幅
	float b;	//脉冲距离
	float tg;	//峰值时间位置
	float tgl;	//半峰时间位置（左)
	float tgr;	//半峰时间位置（右）

	//循环剥离过程
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

		//将该组高斯分量参数压入向量
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

		
	} while (A >3*noise);//循环条件!!!值得探讨


	//对高斯分量做筛选：时间间隔小于一定值的剔除能量较小的分量，将该vector对象的sigma值设为0
	for (int i = 0; i<waveParam.size() - 1; i++)
	{
		for (int j = i + 1; j < waveParam.size(); j++)
		{
			if (abs(waveParam.at(i).b - waveParam.at(j).b) < PulseWidth)//Key
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

	//再将sigma为0值的分量剔除
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


/*功能：			LM算法迭代优化
//&srcWave:		通道原始数据
//&waveParam：	该通道的高斯分量参数
//LM算法参考：	https://blog.csdn.net/shajun0153/article/details/75073137
*/
void WaveData::Optimize(vector<float> &srcWave,vector<GaussParameter> &waveParam)
{
	//解算初值为双峰
	if(waveParam.size()==2)
	{
		//获取高斯函数参数
		double p[6];
		int i = 0;
		for (auto gp : waveParam)
		{
			p[i++] = gp.A;
			p[i++] = gp.b;
			p[i++] = gp.sigma;
		}
		int m = i;
		int n = srcWave.size();

		//获取拟合数据
		double x[320];
		i = 0;
		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
		{
			x[i] = *iter;
		}

		double info[LM_INFO_SZ];
		// 调用迭代入口函数
		int ret = dlevmar_der(expfun2,	//描述测量值之间关系的函数指针
			jacexpfun2,					//估计雅克比矩阵的函数指针
			p,							//初始化的待求参数，结果一并保存在其中
			x,							//测量值
			m,							//参数维度
			n,							//测量值维度
			1000,						//最大迭代次数
			NULL,						//opts,       //迭代的一些参数
			info,						//关于最小化结果的一些参数，不需要设为NULL
			NULL, NULL, NULL			//一些内存的指针，暂时不需要
			);
		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
		printf("波峰时间差: %.7g ns\n", abs(p[4] - p[1]));*/

		//将优化后的参数组赋给vector
		i = 0;
		for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++)
		{
			gaussPraIter->A = p[i++];
			gaussPraIter->b = p[i++];
			gaussPraIter->sigma = p[i++];
		}
	}

	//解算初值为三个峰
	else if (waveParam.size() == 3)
	{
		//获取高斯函数参数
		double p[9];
		int i = 0;
		for (auto gp : waveParam)
		{
			p[i++] = gp.A;
			p[i++] = gp.b;
			p[i++] = gp.sigma;
		}
		int m = i;
		int n = srcWave.size();

		//获取拟合数据
		double x[320];
		i = 0;
		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
		{
			x[i] = *iter;
		}

		double info[LM_INFO_SZ];
		// 调用迭代入口函数
		int ret = dlevmar_der(expfun3,	//描述测量值之间关系的函数指针
			jacexpfun3,					//估计雅克比矩阵的函数指针
			p,							//初始化的待求参数，结果一并保存在其中
			x,							//测量值
			m,							//参数维度
			n,							//测量值维度
			1000,						//最大迭代次数
			NULL,						//opts,       //迭代的一些参数
			info,						//关于最小化结果的一些参数，不需要设为NULL
			NULL, NULL, NULL			//一些内存的指针，暂时不需要
			);
		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
		printf("波峰时间差: %.7g ns\n", abs(p[4] - p[1]));*/

		//将优化后的参数组赋给vector
		i = 0;
		for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++)
		{
			gaussPraIter->A = p[i++];
			gaussPraIter->b = p[i++];
			gaussPraIter->sigma = p[i++];
		}
	}
	
	//解算初值为四个个峰
	else if (waveParam.size() == 4)
	{
		//获取高斯函数参数
		double p[12];
		int i = 0;
		for (auto gp : waveParam)
		{
			p[i++] = gp.A;
			p[i++] = gp.b;
			p[i++] = gp.sigma;
		}
		int m = i;
		int n = srcWave.size();

		//获取拟合数据
		double x[320];
		i = 0;
		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
		{
			x[i] = *iter;
		}

		double info[LM_INFO_SZ];
		// 调用迭代入口函数
		int ret = dlevmar_der(expfun4,	//描述测量值之间关系的函数指针
			jacexpfun4,					//估计雅克比矩阵的函数指针
			p,							//初始化的待求参数，结果一并保存在其中
			x,							//测量值
			m,							//参数维度
			n,							//测量值维度
			1000,						//最大迭代次数
			NULL,						//opts,       //迭代的一些参数
			info,						//关于最小化结果的一些参数，不需要设为NULL
			NULL, NULL, NULL			//一些内存的指针，暂时不需要
			);
		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
		printf("波峰时间差: %.7g ns\n", abs(p[4] - p[1]));*/

		//将优化后的参数组赋给vector
		i = 0;
		for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++)
		{
			gaussPraIter->A = p[i++];
			gaussPraIter->b = p[i++];
			gaussPraIter->sigma = p[i++];
		}
	}

	return;
}


/*功能：	计算水深
//内容：	提取波峰数目小于两个的直接剔除，否则取第一个（即能量最大值）为水面回波，脉冲时间最晚的为水底回波，计算水深
*/
void WaveData::calculateDepth(vector<GaussParameter>& waveParam, float &BorGDepth)
{
	if ((waveParam.size() <= 1)|| (waveParam.size() >= 5))
	{
		BorGDepth=0;
	}
	else
	{
		gaussPraIter = waveParam.begin();
		float tbegin = gaussPraIter->b;
		float tend = tbegin;
		for (gaussPraIter = waveParam.begin()+1; gaussPraIter != waveParam.end(); gaussPraIter++)
		{
			if (gaussPraIter->b > tend)
			{
				tend = gaussPraIter->b;
			}
		}
		//gaussPraIter = waveParam.end()-1;			//!!!坑
		//float tend = gaussPraIter->b;

		BorGDepth = c*(tend - tbegin) / (2 * nwater);
	}
}
;

/*功能：	自定义需要输出的信息
//内容：	年 月 日 时 分 秒 
*/
ostream &operator<<(ostream & stream, const WaveData & wavedata)
{
	stream << wavedata.m_time.year << " "
		<< wavedata.m_time.month << " "
		<< wavedata.m_time.day << " "
		<< wavedata.m_time.hour << " "
		<< wavedata.m_time.minute << " "
		<< wavedata.m_time.second;

	//兴趣数据暂定为制定通道的波峰所在相对位置
	switch (wavedata.ostreamFlag)
	{
	case BLUE: {
		stream << " " << wavedata.blueDepth<<"m";

		if (!wavedata.m_BlueGauPra.empty())
		{
			for (auto p : wavedata.m_BlueGauPra)
			{
				stream << " " << p.b;
			}
		}
		break;
	}
	case GREEN: {
		stream << " " << wavedata.greenDepth << "m";
		if (!wavedata.m_GreenGauPra.empty())
		{
			for (auto p : wavedata.m_GreenGauPra)
			{
				stream << " " << p.b;
			}
		}
		break;
	}
	}

	stream << endl;
	return stream;
}

