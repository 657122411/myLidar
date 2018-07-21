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
	
	unsigned int length;
	//把文件的位置指针移到文件尾获取文件长度
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "Processing:";
	
	
	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("Output.txt", ios::out);

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
			
			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况
			cout.width(3);
			cout << int(100 * 8 * j / length) << "%";
			cout << "\b\b\b\b";

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
		output_stream.close();
		cout << "读取结束！" << endl;
	}
}
