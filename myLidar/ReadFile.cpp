#include "ReadFile.h"

#define BLUE true
#define GREEN false


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

/*功能：	设置读取文件的指针
//in:	读取文件的绝对路径
//out:	将该路径赋给文件指针
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


/*功能：	处理全数据蓝色通道
//out:	读取通道数据滤波去噪分解优化输出
*/
void ReadFile::readBlueAll()
{
	unsigned int j = 0;
	HS_Lidar hs;
	
	//把文件的位置指针移到文件尾获取文件长度
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "BLueChannelProcessing:";
	
	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("BlueOut.txt", ios::out);

	//设置输出流flag
	WaveData::ostreamFlag = BLUE;

	//遍历文件获取数据
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//寻找帧头
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
			mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra,mywave.m_BlueNoise);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
			
			mywave.calculateDepth(mywave.m_BlueGauPra,mywave.blueDepth);

			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length / 800)) << "%";
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
		cout << "finished！" << endl;
	}
}


/*功能：	处理全数据绿色通道
//out:	读取通道数据滤波去噪分解优化输出
*/
void ReadFile::readGreenAll()
{
	unsigned int j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "GreenChannelProcessing:";

	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("GreenOut.txt", ios::out);

	//设置输出流flag
	WaveData::ostreamFlag = GREEN;

	//遍历文件获取数据
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//寻找帧头
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
			mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
			mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
			mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

			mywave.calculateDepth(mywave.m_GreenGauPra, mywave.greenDepth);

			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length/800)) << "%";
			cout << "\b\b\b\b";

		}
		else
		{
			//可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//文件结束退出
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		cout << "finished！" << endl;
	}
}

/*功能：	混合通道处理，选择有效通道
//out:	读取通道数据，根据两通道的标准差大小决定选择相应的通道数据做水深解算
*/
void ReadFile::readMix()
{
	unsigned int j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "MixChannelProcessing:";

	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("MixOut.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//遍历文件获取数据
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//寻找帧头
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

			blueStd = calculateSigma(mywave.m_BlueWave);
			greenStd = calculateSigma(mywave.m_GreenWave);

			blueStd >= 1.2*greenStd ?  bgflag = BLUE : bgflag = GREEN;//判断阈值

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
			

			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length / 800)) << "%";
			cout << "\b\b\b\b";

		}
		else
		{
			//可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//文件结束退出
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		cout << "finished！" << endl;
	}
}


/*功能：	输出原始数据，滤波后数据，初始解数据，迭代后数据
//out:	4files
*/
void ReadFile::outputData()
{
	unsigned int j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "MixChannelProcessing:";

	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("MixOut.txt", ios::out);


	fstream origin;//初始数据
	fstream filter;//滤波数据
	fstream resolve;//初解算数据
	fstream iterate;//迭代数据
	origin.open("origin.txt", ios::out);
	filter.open("filter.txt", ios::out);
	resolve.open("resolve.txt", ios::out);
	iterate.open("iterate.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//遍历文件获取数据
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//寻找帧头
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

			blueStd = calculateSigma(mywave.m_BlueWave);
			greenStd = calculateSigma(mywave.m_GreenWave);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

			switch (bgflag)
			{
			case BLUE:
				WaveData::ostreamFlag = BLUE;

				//输出原始数据
				for (auto data : mywave.m_BlueWave)
				{
					origin << data << " ";
				}
				origin << endl;
	
				mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);

				//输出滤波数据
				for (auto data : mywave.m_BlueWave)
				{
					filter << data << " ";
				}
				filter << endl;

				mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);

				//输出初解数据
				for (auto data : mywave.m_BlueGauPra)
				{
					resolve << data.A << " "<<data.b<<" "<<data.sigma<<" ";
				}
				resolve << endl;

				mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

				//输出迭代数据
				for (auto data : mywave.m_BlueGauPra)
				{
					iterate << data.A << " " << data.b << " " << data.sigma << " ";
				}
				iterate << endl;

				mywave.calculateDepth(mywave.m_BlueGauPra, mywave.blueDepth);
				break;
			case GREEN:
				WaveData::ostreamFlag = GREEN;

				//输出滤波数据
				for (auto data : mywave.m_GreenWave)
				{
					filter << data << " ";
				}
				filter << endl;

				mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);

				//输出滤波数据
				for (auto data : mywave.m_GreenWave)
				{
					filter << data << " ";
				}
				filter << endl;

				mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);

				//输出初解数据
				for (auto data : mywave.m_GreenGauPra)
				{
					resolve << data.A << " " << data.b << " " << data.sigma << " ";
				}
				resolve << endl;

				mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

				//输出迭代数据
				for (auto data : mywave.m_GreenGauPra)
				{
					iterate << data.A << " " << data.b << " " << data.sigma << " ";
				}
				iterate << endl;

				mywave.calculateDepth(mywave.m_GreenGauPra, mywave.greenDepth);
				break;
			default:
				break;
			}


			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length / 800)) << "%";
			cout << "\b\b\b\b";

		}
		else
		{
			//可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//文件结束退出
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		origin.close();//初始数据
		filter.close();//滤波数据
		resolve.close();//初解算数据
		iterate.close();//迭代数据
		cout << "finished！" << endl;
	}

}


/*功能：	读取深水数据
//out:	
*/
void ReadFile::readDeep()
{
	unsigned int j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned int length;
	fseek(m_filePtr, 0L, SEEK_END);
	length = ftell(m_filePtr);
	cout << "ReadDeepProcessing:";


	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("DeepOut.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;



	//遍历文件获取数据
	do {
		_fseeki64(m_filePtr, j * 8, SEEK_SET);

		//寻找帧头
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//处理数据的流程：
			_fseeki64(m_filePtr, -8, SEEK_CUR);
			hs.initDeepData(m_filePtr);

			//获取通道的深水段回波数据
			DeepWave dw;
			dw.GetDeepData(hs);

			//process
			blueStd = calculateSigma(dw.m_BlueDeep);
			greenStd = calculateSigma(dw.m_GreenDeep);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

			switch (bgflag)
			{
			case BLUE:
				DeepWave::ostreamFlag = BLUE;

				dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepGauPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepGauPra);

				dw.calculateDeepDepth(dw.m_BlueDeepGauPra, dw.blueDeepDepth);
				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepGauPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepGauPra);

				dw.calculateDeepDepth(dw.m_GreenDeepGauPra, dw.greenDeepDepth);
				break;
			default:
				break;
			}


			//输出信息到文件
			output_stream << dw;




			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况
			cout.width(3);
			cout << int(/*100 * 8 **/ j / (length / 800)) << "%";
			cout << "\b\b\b\b";

		}
		else
		{
			//可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//文件结束退出
	if (feof(m_filePtr) == 1)
	{
		cout << "finished！" << endl;
	}
}