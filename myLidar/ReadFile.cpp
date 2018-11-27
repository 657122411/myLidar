#include "ReadFile.h"

#define BLUE true
#define GREEN false


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

/*���ܣ�	���ö�ȡ�ļ���ָ��
//in:	��ȡ�ļ��ľ���·��
//out:	����·�������ļ�ָ��
*/
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


/*���ܣ�	����ȫ������ɫͨ��
//out:	��ȡͨ�������˲�ȥ��ֽ��Ż����
*/
void ReadFile::readBlueAll()
{
	unsigned int j = 0;
	HS_Lidar hs;
	
	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "BLueChannelProcessing:";
	
	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("BlueOut.txt", ios::out);

	//���������flag
	WaveData::ostreamFlag = BLUE;

	//�����ļ���ȡ����
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
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
			mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra,mywave.m_BlueNoise);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
			
			mywave.calculateDepth(mywave.m_BlueGauPra,mywave.blueDepth);

			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ����������
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length / 800)) << "%";
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
		cout << "finished��" << endl;
	}
}


/*���ܣ�	����ȫ������ɫͨ��
//out:	��ȡͨ�������˲�ȥ��ֽ��Ż����
*/
void ReadFile::readGreenAll()
{
	unsigned int j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "GreenChannelProcessing:";

	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("GreenOut.txt", ios::out);

	//���������flag
	WaveData::ostreamFlag = GREEN;

	//�����ļ���ȡ����
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
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
			mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
			mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
			mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

			mywave.calculateDepth(mywave.m_GreenGauPra, mywave.greenDepth);

			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ����������
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length/800)) << "%";
			cout << "\b\b\b\b";

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		cout << "finished��" << endl;
	}
}

/*���ܣ�	���ͨ������ѡ����Чͨ��
//out:	��ȡͨ�����ݣ�������ͨ���ı�׼���С����ѡ����Ӧ��ͨ��������ˮ�����
*/
void ReadFile::readMix()
{
	unsigned int j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "MixChannelProcessing:";

	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("MixOut.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//�����ļ���ȡ����
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
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

			blueStd = calculateSigma(mywave.m_BlueWave);
			greenStd = calculateSigma(mywave.m_GreenWave);

			blueStd >= 1.2*greenStd ?  bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

			switch (bgflag)
			{
			case BLUE:
				WaveData::ostreamFlag = BLUE;

				mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
				mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
				mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

				mywave.calculateDepth(mywave.m_BlueGauPra, mywave.blueDepth);
				break;
			case GREEN:
				WaveData::ostreamFlag = GREEN;

				mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
				mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
				mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

				mywave.calculateDepth(mywave.m_GreenGauPra, mywave.greenDepth);
				break;
			default:
				break;
			}
			

			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ����������
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length / 800)) << "%";
			cout << "\b\b\b\b";

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		cout << "finished��" << endl;
	}
}