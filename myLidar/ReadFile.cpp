#include "ReadFile.h"

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
	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "Processing:";
	
	
	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("Output.txt", ios::out);

	//�����ļ���ȡ����
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);


		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			_fseeki64(m_filePtr, -8, SEEK_CUR);
			hs.initData(m_filePtr);

			WaveData mywave;
			mywave.GetData(hs);
			mywave.Filter(mywave.m_BlueWave);
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
			
			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�����������
			cout.width(3);
			cout << int(100 * 8 * j / length) << "%";
			cout << "\b\b\b\b";

		}
		else
		{	
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j+=2;
		}
		
	} while (!feof(m_filePtr));


	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		cout << "��ȡ������" << endl;
	}
}