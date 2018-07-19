// myLidar.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "WaveData.h"
using namespace std;


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


int main()
{
	char filename[100];

	cout << "��������Ҫ�򿪵��ļ�·������c:\\temp.txt��" << endl;
	cin >> filename;
	FILE * mf;
	mf = fopen(filename, "rb");
	if (mf == NULL)
	{
		printf("��ȡ�ļ�����");
		return 0;
	}

	unsigned int j = 0;
	HS_Lidar hs;
	//�����ļ���ȡ����
	do {
		_fseeki64(mf, j * 8, SEEK_SET);

		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, mf);
		if (isHeaderRight(header))
		{
			_fseeki64(mf, -8, SEEK_CUR);
			hs.initData(mf);
			WaveData mywave;
			mywave.GetData(hs);
			mywave.Filter(mywave.m_BlueWave);
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
		}

		j++;
	} while (!feof(mf));

	//�ļ������˳�
	if (feof(mf) == 1)
	{
		cout << "��ȡ������" << endl;
		system("pause");
		exit(0);
	}

    return 0;
}

