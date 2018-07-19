#include "WaveData.h"


//�ж�֡ͷ�Ƿ���ȷ
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


/*���ܣ� ��˹ģ��
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


WaveData::WaveData()
{

};

WaveData::~WaveData()
{

};


/*��ȡԭʼ�����е���Ȥ��������*/
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


/*ȥ���˲�����*/
void WaveData::Filter(vector<float> &srcWave)
{
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	gaussian(&srcWave[0], &dstWave[0]);
	srcWave.assign(dstWave.begin(), dstWave.end());
	dstWave.clear();
};


/*��˹�����ֽ⺯��*/
void WaveData::Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam)
{
	float data[320],temp[320];
	int i = 0, m = 0;
	for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
	{
		data[i] = *iter;
	}

	float min = data[0];//������Сֵ
	for (m = 0; m < 320; m++)
	{
		temp[m] = data[m];
		if (data[m] < min)	
			min = data[m];
	}

	//ȥ��
	for (m = 0; m < 320; m++)
	{
		temp[m] -= min;
	}

	srcWave.assign(&temp[0],&temp[320]);

	//��˹����
	float A;//���
	float b, tg, tgl, tgr;//������룬��ֵʱ��λ�ã����ʱ��λ�ã����ң�


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





	//�Ը�˹������ɸѡ��ʱ����С��һ��ֵ���޳�������С�ķ�����
	//�Ƚ���ֵ��Сvector�����sigmaֵ��Ϊ0
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

	//�ٽ�sigmaΪ0ֵ�ķ���ɾ��
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



