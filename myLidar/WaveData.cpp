#include "WaveData.h"

/*���ܣ�  ��˹������
//kernel���洢���ɵĸ�˹��
//size��  �˵Ĵ�С
//sigma�� ��̬�ֲ���׼��
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


/*����:	 ��˹ģ��
//src��  ����ԭͼ
//dst��  ģ��ͼ��
//size�� �˵Ĵ�С
//sigma����̬�ֲ���׼��
*/
void gaussian(float src[], float dst[])
{
	float kernel[5];
	gau_kernel(kernel, 5, 1);
	//gaussian���,��ʱ�߽�û�Ӵ���
	for (int i = (5 - 1) / 2; i <= 319 - (5 - 1) / 2; i++)
	{
		dst[i] = src[i - 2] * kernel[0] + src[i - 1] * kernel[1] + src[i] * kernel[2] + src[i + 1] * kernel[3] + src[i + 2] * kernel[4];
	}
}


/*���ܣ�	���躯��ģ��
//*p:	�������
//*x��  ԭʼ���ݣ�����ֵ��
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void expfun(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2])) + p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]));
	}

}


/*���ܣ�	����ģ�͵��ſɱȾ���
//*p:	�������
//jac�� �ſɱȾ������
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void jacexpfun(double *p, double *jac, int m, int n, void *data)
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


WaveData::WaveData()
{

};

WaveData::~WaveData()
{

};


/*���ܣ�	��ȡԭʼ�����е���Ȥ��������
//*&hs:	ԭʼLidar����
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
	m_time.hour = pct->hour+8;	//ֱ��ת��Ϊ����ʱ��
	m_time.minute = pct->minute;
	m_time.second = pct->second;
	delete pgt;
	delete pct;

	//ȡ����ͨ��
	m_BlueWave.assign(&hs.CH2.nD0[0], &hs.CH2.nD0[320]);
	m_GreenWave.assign(&hs.CH3.nD0[0], &hs.CH3.nD0[320]);

};


/*���ܣ�		ȥ���˲�����
//&srcWave:	ͨ��ԭʼ����
*/
void WaveData::Filter(vector<float> &srcWave)
{
	//��˹�˲�ȥ��
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	gaussian(&srcWave[0], &dstWave[0]);
	srcWave.assign(dstWave.begin(), dstWave.end());
	dstWave.clear();
};


/*���ܣ�			��˹�����ֽ⺯��
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
*/
void WaveData::Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam)
{
	//����ԭʼ����
	float data[320],temp[320];
	int i = 0, m = 0;
	for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
	{
		data[i] = *iter;
	}
	
	//����������СֵΪ��������
	float min = data[0];
	for (m = 0; m < 320; m++)
	{
		temp[m] = data[m];
		if (data[m] < min)	
			min = data[m];
	}

	//�������ݳ�ȥ��������
	for (m = 0; m < 320; m++)
	{
		temp[m] -= min;
	}
	srcWave.assign(&temp[0],&temp[320]);

	float A;	//���
	float b;	//�������
	float tg;	//��ֵʱ��λ��
	float tgl;	//���ʱ��λ�ã���)
	float tgr;	//���ʱ��λ�ã��ң�

	//ѭ���������
	do
	{
		A = 0;
		//�����ֵ����¼λ��
		for (m = 0; m < 320; m++)
		{
			if (temp[m] > A)
			{
				A = temp[m];
				b = m;
			}
		}

		//Ѱ�Ұ��λ��
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

		//����sigma
		float sigma = fabs(tg - b) / sqrt(2 * log(2));

		//�������˹��������ѹ������
		GaussParameter param{A,b,sigma};
		waveParam.push_back(param);

		//����
		for (m = 0; m < 320; m++)
		{
			if (temp[m] > A*exp(-(m - b)*(m - b) / (2 * sigma*sigma)))
			{
				temp[m] -= A*exp(-(m - b)*(m - b) / (2 * sigma*sigma));
			}
			else
				temp[m] = 0;
		}

		//�ж��Ƿ��������
		A = 0;
		for (m = 0; m < 320; m++)
		{
			if (temp[m] > A)
			{
				A = temp[m];
			}
		}

		//��ȡ����������ṹ��ĵ�һ������ֵ����ֵ�ο���
		gaussPraIter = waveParam.begin();
	} while (A >= 1.5*20/*Gnoise*/);//ѭ������!!!ֵ��̽��


	//�Ը�˹������ɸѡ��ʱ����С��һ��ֵ���޳�������С�ķ���������vector�����sigmaֵ��Ϊ0
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

	//�ٽ�sigmaΪ0ֵ�ķ����޳�
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


/*���ܣ�			LM�㷨�����Ż�
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
//LM�㷨�ο���	https://blog.csdn.net/shajun0153/article/details/75073137
*/
void WaveData::Optimize(vector<float> &srcWave,vector<GaussParameter> &waveParam)
{
	//����ĸ�˹���������������������Ż�
	if (waveParam.size()!=2)
	{
		return;
	}

	//��ȡ��˹��������
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

	//��ȡ�������
	double x[320];
	i = 0;
	for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
	{
		x[i] = *iter;
	}
	
	double info[LM_INFO_SZ];
	// ���õ�����ں���
	int ret = dlevmar_der(expfun,	//��������ֵ֮���ϵ�ĺ���ָ��
		jacexpfun,					//�����ſ˱Ⱦ���ĺ���ָ��
		p,							//��ʼ���Ĵ�����������һ������������
		x,							//����ֵ
		m,							//����ά��
		n,							//����ֵά��
		1000,						//����������
		NULL,						//opts,       //������һЩ����
		info,						//������С�������һЩ����������Ҫ��ΪNULL
		NULL, NULL, NULL			//һЩ�ڴ��ָ�룬��ʱ����Ҫ
		);
	printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
	printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
	printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));

	//���Ż���Ĳ����鸳��vector
	i = 0;
	for (auto gp : waveParam)
	{
		gp.A = p[i++];
		gp.b = p[i++];
		gp.sigma = p[i++];
	}

};



