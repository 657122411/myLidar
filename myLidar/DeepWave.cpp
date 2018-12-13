#include "DeepWave.h"
#include <numeric>
#include <algorithm>

#define DeepPulseWidth 4		//���弤����������������ֵ�ο�
#define TimeDifference 8	//��UTC��ʱ��

#define BLUE true
#define GREEN false

#define c 0.3				//��Թ��ٳ�������
#define ndeepwater 1.34		//��ˮˮ�ʵ�������

// �ز�����
#define DEEPSURFACE true		//ˮ��ز������ܰ�������ɢ�䣩
#define DEEPBOTTOM false		//ˮ�׻�ˮ�����ʻز�

bool DeepWave::ostreamFlag = BLUE;

/*���ܣ�  ��˹������
//kernel���洢���ɵĸ�˹��
//size��  �˵Ĵ�С
//sigma�� ��̬�ֲ���׼��
*/
void deep_gau_kernel(float kernel[], int size, float sigma)
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


/*����:	 ��˹ģ��
//src��  ����ԭͼ
//dst��  ģ��ͼ��
//size�� �˵Ĵ�С
//sigma����̬�ֲ���׼��
*/
void deep_gaussian(float src[], float dst[])
{
	float kernel[7];
	deep_gau_kernel(kernel, 7, 1);
	//gaussian���,��ʱ�߽�û�Ӵ���
	for (int i = (7 - 1) / 2; i <= 799 - (7 - 1) / 2; i++)
	{
		dst[i] = src[i - 3] * kernel[0] + src[i - 2] * kernel[1] + src[i - 1] * kernel[2] + src[i] * kernel[3] + src[i + 1] * kernel[4] + src[i + 2] * kernel[5] + src[i + 3] * kernel[6];
	}
}


/*���ܣ�	�������ݵı�׼��
//*:
//resultSet���������������
//stdev��	����ֵΪ��׼��
//*
*/
float calculateDeepSigma(vector<float> resultSet)
{
	double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
	double mean = sum / resultSet.size(); //��ֵ  

	double accum = 0.0;
	for each (float d in resultSet)
	{
		accum += (d - mean)*(d - mean);
	}

	float stdev = sqrt(accum / (resultSet.size() - 1)); //����  

	return stdev;

}



/*���ܣ�	���������˹����ģ��
//*p:	�������
//*x��  ԭʼ���ݣ�����ֵ��
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_expfun2(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]));
	}

}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
//*p:	�������
//jac�� �ſɱȾ������
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_jacexpfun2(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//д���ſ˱Ⱦ���
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


/*���ܣ�	���������˹����ģ��
//*p:	�������
//*x��  ԭʼ���ݣ�����ֵ��
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_expfun3(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]))
			+ p[6] * exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*(2 * p[8]));
	}

}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
//*p:	�������
//jac�� �ſɱȾ������
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_jacexpfun3(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//д���ſ˱Ⱦ���
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


/*���ܣ�	���������˹����ģ��
//*p:	�������
//*x��  ԭʼ���ݣ�����ֵ��
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_expfun4(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]))
			+ p[6] * exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*(2 * p[8]))
			+ p[9] * exp(-(i - p[10])*(i - p[10]) / (2 * p[11])*(2 * p[11]));
	}

}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
//*p:	�������
//jac�� �ſɱȾ������
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_jacexpfun4(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//д���ſ˱Ⱦ���
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



DeepWave::DeepWave()
{
	m_time = { 0,0,0,0,0,0 };
	m_BlueDeepNoise = 0;
	m_GreenDeepNoise = 0;
	blueDeepDepth = 0;
	greenDeepDepth = 0;
}

DeepWave::~DeepWave()
{
}

void DeepWave::GetDeepData(HS_Lidar & hs)
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
	m_time.hour = pct->hour + TimeDifference;	//ֱ��ת��Ϊ����ʱ��
	m_time.minute = pct->minute;
	m_time.second = pct->second;
	delete pgt;
	delete pct;

	//ȡ����ͨ����ˮ����
	vector<int >::iterator it;//����������
	for (it = hs.deepData2.begin(); it != hs.deepData2.end(); ++it) 
	{
		m_BlueDeep.push_back((float)*it);
	}
	for (it = hs.deepData3.begin(); it != hs.deepData3.end(); ++it) 
	{
		m_GreenDeep.push_back((float)*it);
	}
}


/*���ܣ�		Ԥ�������ݣ���ȡ��Ч���ֲ�����ȥ���˲�����
//&srcWave:	ͨ��ԭʼ����
//&noise��	��¼��������������
*/
void DeepWave::DeepFilter(vector<float> &srcWave, float &noise)
{
	//��˹�˲�ȥ��
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	deep_gaussian(&srcWave[0], &dstWave[0]);

	noise = 0;
	//�����������:�����˲�ǰ��Ĳ������ݵķ�ֵ��ľ������׼�
	for (int i = 0; i < srcWave.size(); i++)
	{
		noise += (srcWave.at(i) - dstWave.at(i)) * (srcWave.at(i) - dstWave.at(i));
	}
	noise = sqrt(noise / srcWave.size());

	srcWave.assign(dstWave.begin(), dstWave.end());
}


/*���ܣ�			��˹�����ֽ⺯��
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
*/
void DeepWave::DeepResolve(vector<float> &srcWave, vector<DeepGaussParameter> &waveParam, float &noise)
{
	//����ԭʼ����
	vector<float> data,temp;
	data.assign(srcWave.begin(), srcWave.end());

	//���˲����������Сֵ��Ϊ��������
	vector<float>::iterator smallest = min_element(begin(data), end(data));
	float backgroundNoise = *smallest;

	//�������ݳ�ȥ��������
	for (vector<float>::iterator m = data.begin(); m != data.end(); m++) //�õ������ķ�ʽ
	{
		*m -= backgroundNoise;
	}

	float A;	//���
	float b;	//�������
	float tg;	//��ֵʱ��λ��
	float tgl;	//���ʱ��λ�ã���)
	float tgr;	//���ʱ��λ�ã��ң�

	bool wavetypeFlag = true;			//�����ж�ˮ��ˮ�׻ز������flag
	float surfaceMin, surfaceMax;	//ˮ��ز�λ�����ڵĿ��Ʒ�Χ

									//ѭ���������
	do
	{
		A = 0;
		//�����ֵ����¼λ��
		vector<float>::iterator biggest = max_element(begin(data), end(data));
		A = *biggest;
		b = distance(data.begin(), biggest);


		//Ѱ�Ұ��λ��
		for (int m = b; m < data.size(); m++)
		{
			if ((data.at(m - 1) > A / 2) && (data.at(m + 1) < A / 2))
			{
				tgr = m;
				break;
			}
		}
		for (int m = b; m > 0; m--)
		{
			if ((data.at(m - 1) < A / 2) && (data.at(m + 1) > A / 2))
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

		//����sigma
		float sigma = fabs(tg - b) / sqrt(2 * log(2));

		//�ж�ˮ��ˮ�׻ز�
		if (wavetypeFlag == true)
		{
			//���Ҳ�յ�
			float rval = abs(data.at((int)b + 1) - data.at((int)b));
			for (int i = b; b < data.size() - b; i++)
			{
				if (abs(data.at(i + 1) - data.at(i)) >= rval)
					rval = abs(data.at(i + 1) - data.at(i));
				else
				{
					surfaceMax = i + 2;
					break;
				}
			}
			//�����յ�
			float lval = abs(data.at((int)b - 1) - data.at((int)b));
			for (int i = b; b > 0; i--)
			{
				if (abs(data.at(i - 1) - data.at(i)) >= lval)
					lval = abs(data.at(i - 1) - data.at(i));
				else
				{
					surfaceMin = i - 2;
					break;
				}
			}

			wavetypeFlag = false;

		}

		if (surfaceMin <= b&& b <= surfaceMax)//��ѡ�������ڵĸ�˹����Ϊͬһ�������ʵˮ��+����ɢ�䣩
		{
			//�������˹��������ѹ������
			DeepGaussParameter param{ A,b,sigma,DEEPSURFACE };
			waveParam.push_back(param);
		}
		else
		{
			//�������˹��������ѹ������
			DeepGaussParameter param{ A,b,sigma,DEEPBOTTOM };
			waveParam.push_back(param);
		}

		//����
		for (int m = 0; m < data.size(); m++)
		{
			if (data.at(m) > A*exp(-(m - b)*(m - b) / (2 * sigma*sigma)))
			{
				data.at(m) -= A*exp(-(m - b)*(m - b) / (2 * sigma*sigma));
			}
			else
				data.at(m) = 0;
		}

		//�ж��Ƿ��������
		A = 0;
		for (int m = 0; m < data.size(); m++)
		{
			if (data.at(m) > A)
			{
				A = data.at(m);
			}
		}


	} while (A >5 * noise);//ѭ������!!!ֵ��̽��


						   //�Ը�˹������ɸѡ��ʱ����С��һ��ֵ���޳�������С�ķ���������vector�����sigmaֵ��Ϊ0
	for (int i = 0; i<waveParam.size() - 1; i++)
	{
		for (int j = i + 1; j < waveParam.size(); j++)
		{
			if (abs(waveParam.at(i).Db - waveParam.at(j).Db) < DeepPulseWidth)//Key
			{
				if (waveParam.at(i).DA >= waveParam.at(j).DA)
				{
					waveParam.at(j).Dsigma = 0;
				}
				else
				{
					waveParam.at(i).Dsigma = 0;
				}
			}
		}
	}

	//�ٽ�sigmaС����ֵ�ķ����޳�
	for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end();)
	{
		if (DeepgaussPraIter->Dsigma < ((float)DeepPulseWidth / 8))
		{
			DeepgaussPraIter = waveParam.erase(DeepgaussPraIter);
		}
		else
		{
			++DeepgaussPraIter;
		}
	}
}


/*���ܣ�			LM�㷨�����Ż�
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
//LM�㷨�ο���	https://blog.csdn.net/shajun0153/article/details/75073137
*/
void DeepWave::DeepOptimize(vector<float> &srcWave, vector<DeepGaussParameter> &waveParam)
{
	//�����ֵΪ˫��
	if (waveParam.size() == 2)
	{
		//��ȡ��˹��������
		double p[6];
		int i = 0;
		for (auto gp : waveParam)
		{
			p[i++] = gp.DA;
			p[i++] = gp.Db;
			p[i++] = gp.Dsigma;
		}
		int m = i;
		int n = srcWave.size();

		//��ȡ�������
		double x[800];
		i = 0;
		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
		{
			x[i] = *iter;
		}

		double info[LM_INFO_SZ];
		// ���õ�����ں���
		int ret = dlevmar_der(deep_expfun2,	//��������ֵ֮���ϵ�ĺ���ָ��
			deep_jacexpfun2,					//�����ſ˱Ⱦ���ĺ���ָ��
			p,							//��ʼ���Ĵ�����������һ������������
			x,							//����ֵ
			m,							//����ά��
			n,							//����ֵά��
			1000,						//����������
			NULL,						//opts,       //������һЩ����
			info,						//������С�������һЩ����������Ҫ��ΪNULL
			NULL, NULL, NULL			//һЩ�ڴ��ָ�룬��ʱ����Ҫ
		);
		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
		printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

		//���Ż���Ĳ����鸳��vector
		i = 0;
		for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
		{
			DeepgaussPraIter->DA = p[i++];
			DeepgaussPraIter->Db = p[i++];
			DeepgaussPraIter->Dsigma = p[i++];
		}
	}

	//�����ֵΪ������
	else if (waveParam.size() == 3)
	{
		//��ȡ��˹��������
		double p[9];
		int i = 0;
		for (auto gp : waveParam)
		{
			p[i++] = gp.DA;
			p[i++] = gp.Db;
			p[i++] = gp.Dsigma;
		}
		int m = i;
		int n = srcWave.size();

		//��ȡ�������
		double x[800];
		i = 0;
		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
		{
			x[i] = *iter;
		}

		double info[LM_INFO_SZ];
		// ���õ�����ں���
		int ret = dlevmar_der(deep_expfun3,	//��������ֵ֮���ϵ�ĺ���ָ��
			deep_jacexpfun3,					//�����ſ˱Ⱦ���ĺ���ָ��
			p,							//��ʼ���Ĵ�����������һ������������
			x,							//����ֵ
			m,							//����ά��
			n,							//����ֵά��
			1000,						//����������
			NULL,						//opts,       //������һЩ����
			info,						//������С�������һЩ����������Ҫ��ΪNULL
			NULL, NULL, NULL			//һЩ�ڴ��ָ�룬��ʱ����Ҫ
		);
		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
		printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

		//���Ż���Ĳ����鸳��vector
		i = 0;
		for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
		{
			DeepgaussPraIter->DA = p[i++];
			DeepgaussPraIter->Db = p[i++];
			DeepgaussPraIter->Dsigma = p[i++];
		}
	}

	//�����ֵΪ�ĸ�����
	else if (waveParam.size() == 4)
	{
		//��ȡ��˹��������
		double p[12];
		int i = 0;
		for (auto gp : waveParam)
		{
			p[i++] = gp.DA;
			p[i++] = gp.Db;
			p[i++] = gp.Dsigma;
		}
		int m = i;
		int n = srcWave.size();

		//��ȡ�������
		double x[800];
		i = 0;
		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
		{
			x[i] = *iter;
		}

		double info[LM_INFO_SZ];
		// ���õ�����ں���
		int ret = dlevmar_der(deep_expfun4,	//��������ֵ֮���ϵ�ĺ���ָ��
			deep_jacexpfun4,					//�����ſ˱Ⱦ���ĺ���ָ��
			p,							//��ʼ���Ĵ�����������һ������������
			x,							//����ֵ
			m,							//����ά��
			n,							//����ֵά��
			1000,						//����������
			NULL,						//opts,       //������һЩ����
			info,						//������С�������һЩ����������Ҫ��ΪNULL
			NULL, NULL, NULL			//һЩ�ڴ��ָ�룬��ʱ����Ҫ
		);
		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
		printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

		//���Ż���Ĳ����鸳��vector
		i = 0;
		for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
		{
			DeepgaussPraIter->DA = p[i++];
			DeepgaussPraIter->Db = p[i++];
			DeepgaussPraIter->Dsigma = p[i++];
		}
	}

	return;
}


/*���ܣ�	����ˮ��
//���ݣ�	��ȡ������ĿС��������ֱ���޳�������ȡ��һ�������������ֵ��Ϊˮ��ز�������ʱ�������Ϊˮ�׻ز�������ˮ��
*/
void DeepWave::calculateDeepDepth(vector<DeepGaussParameter>& waveParam, float &BorGDepth)
{
	if ((waveParam.size() <= 1) || (waveParam.size() >= 5))
	{
		BorGDepth = 0;
	}
	else
	{
		DeepgaussPraIter = waveParam.begin();
		float tbegin = DeepgaussPraIter->Db;
		float tend = tbegin;

		for (DeepgaussPraIter = waveParam.begin() + 1; DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
		{

			if ((DeepgaussPraIter->Db > tend) && (DeepgaussPraIter->deepwavetype == DEEPBOTTOM))//ˮ�׻ز��ض�������ˮ��ز��ĺ���ʱ�̣�Ϊ��ײ������������𣬼ٶ�����ˮ��ز��Ļز�ʱ�������������ڣ�����ˮ�����ɢ�䣩
			{
				tend = DeepgaussPraIter->Db;
				break;
			}
		}
		//gaussPraIter = waveParam.end()-1;			//!!!��
		//float tend = gaussPraIter->b;

		BorGDepth = c*(tend - tbegin) / (2 * ndeepwater);
	}
}


/*���ܣ�	�Զ�����Ҫ�������Ϣ
//���ݣ�	�� �� �� ʱ �� ��
*/
ostream &operator<<(ostream & stream, const DeepWave & wavedata)
{
	stream << wavedata.m_time.year << " "
		<< wavedata.m_time.month << " "
		<< wavedata.m_time.day << " "
		<< wavedata.m_time.hour << " "
		<< wavedata.m_time.minute << " "
		<< wavedata.m_time.second;

	//��Ȥ�����ݶ�Ϊ�ƶ�ͨ���Ĳ����������λ��
	switch (wavedata.ostreamFlag)
	{
	case BLUE: {
		stream << " " << wavedata.blueDeepDepth << "m";

		if (!wavedata.m_BlueDeepGauPra.empty())
		{
			for (auto p : wavedata.m_BlueDeepGauPra)
			{
				stream << " " << p.Db;
			}
		}
		break;
	}
	case GREEN: {
		stream << " " << wavedata.greenDeepDepth << "m";
		if (!wavedata.m_GreenDeepGauPra.empty())
		{
			for (auto p : wavedata.m_GreenDeepGauPra)
			{
				stream << " " << p.Db;
			}
		}
		break;
	}
	}

	stream << endl;
	return stream;
}