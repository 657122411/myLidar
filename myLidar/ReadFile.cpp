#include "ReadFile.h"

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

ReadFile::ReadFile()
{
}

ReadFile::~ReadFile()
{
}

bool ReadFile::setFilename(char filename[100])
{
	m_filename = filename;
	m_filePtr = fopen(m_filename, "rb");
	if (m_filePtr == NULL)
	{
		cout << "file loading failed!" << endl;
		return false;
	}
	else
	{
		cout << "file loading successed!" << endl;
		return true;
	}
}

void ReadFile::readAll()
{
	unsigned int j = 0;
	HS_Lidar hs;
	//遍历文件获取数据
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);

		if (isHeaderRight(header))
		{
			//处理数据的流程：
			_fseeki64(m_filePtr, -8, SEEK_CUR);
			hs.initData(m_filePtr);

			WaveData mywave;
			mywave.GetData(hs);
			mywave.Filter(mywave.m_BlueWave);
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
			cout << mywave;


			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;
		}
		else
		{	
			//可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
			j+=2;
		}
		
	} while (!feof(m_filePtr));

	//文件结束退出
	if (feof(m_filePtr) == 1)
	{
		cout << "读取结束！" << endl;
		system("pause");
		exit(0);
	}
}
