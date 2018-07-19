// myLidar.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "WaveData.h"

int main()
{
	char filename[100];

	cout << "请输入你要打开的文件路径，如c:\\temp.txt：" << endl;
	cin >> filename;
	FILE * mf;
	mf = fopen(filename, "rb");
	if (mf == NULL)
	{
		printf("读取文件出错");
		return 0;
	}

	unsigned int j = 0;
	HS_Lidar hs;
	//遍历文件获取数据
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

		}

		j++;
	} while (!feof(mf));

	//文件结束退出
	if (feof(mf) == 1)
	{
		cout << "读取结束！" << endl;
		system("pause");
		exit(0);
	}

    return 0;
}

