/*************************************************
Author:陶剑浩
Date:2019-01-02
Description:测深数据文件操作类
**************************************************/
#include "ReadFile.h"

#define BLUE true
#define GREEN false


//判断两浮点数是否相等(经纬度变化小于绝对误差)
bool isEqual(const double a, const double b) 
{
	const double eps_0 = 1.0e-6;
	bool isEqualFlag = false;
	if (fabs(a - b) <= eps_0) {
		isEqualFlag = true;
	}

	return isEqualFlag;
}


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


/*************************************************
Function:       设置读取文件的指针
Description:    
Input:          读取文件的绝对路径
Output:			将该路径赋给文件指针
*************************************************/
bool ReadFile::setFilename(char filename[100])
{
	m_filename = filename;
	m_filePtr = fopen(m_filename, "rb");
	if (m_filePtr == NULL)
	{
		printf("\nFile load failed!\n");
		return false;
	}
	else
	{
		printf("\nFile loaded successfully!\n");
		return true;
	}
}


/*************************************************
Function:       处理全数据蓝色通道
Description:	读取通道数据滤波去噪分解优化输出
Input:          
Output:			CH2水深解算结果
*************************************************/
void ReadFile::readBlueAll()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;
	
	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("BLueChannelProcessing:");
	
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
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

			mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);

			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		printf("Finished!\n");
	}
}


/*************************************************
Function:       处理全数据绿色通道
Description:	读取通道数据滤波去噪分解优化输出
Input:
Output:			CH3水深解算结果
*************************************************/
void ReadFile::readGreenAll()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("GreenChannelProcessing:");

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

			mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);

			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		printf("Finished!\n");
	}
}


/*************************************************
Function:       全数据混合通道处理
Description:	读取通道数据，根据两通道的标准差大小决定选择相应的通道数据做水深解算
Input:
Output:			CH2,CH3通道有效数据水深解算结果
*************************************************/
void ReadFile::readMix()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("MixChannelProcessing:");

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

				mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);
				break;
			case GREEN:
				WaveData::ostreamFlag = GREEN;

				mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
				mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
				mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

				mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);
				break;
			default:
				break;
			}
			
			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		printf("Finished!\n");
	}
}


/*************************************************
Function:       全数据混合通道处理并步骤输出
Description:	输出原始数据，滤波后数据，初始解数据，迭代后数据，水深解算数据
Input:
Output:			CH2,CH3通道有效数据水深解算各步骤结果
*************************************************/
void ReadFile::outputData()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("MixChannelProcessing:");

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

				mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);
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

				mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);
				break;
			default:
				break;
			}

			//输出信息到文件
			output_stream << mywave;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		printf("Finished!\n");
	}

}


/*************************************************
Function:       深水数据CH2CH3通道混合处理
Description:	读取通道数据，根据两通道的标准差大小决定选择相应的通道数据做水深解算
Input:
Output:			CH2,CH3通道有效数据水深解算结果
*************************************************/
void ReadFile::readDeep()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("ReadDeepProcessing:");


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
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

				dw.CalcuDeepDepth(dw.m_BlueDeepPra, dw.blueDeepDepth);
				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepPra);

				dw.CalcuDeepDepth(dw.m_GreenDeepPra, dw.greenDeepDepth);
				break;
			default:
				break;
			}

			//输出信息到文件
			output_stream << dw;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		printf("Finished!\n");
	}
}


/*************************************************
Function:       深水数据CH1，CH2CH3通道混合处理
Description:	近红外通道确定水面，蓝绿通道的标准差大小决定选择相应的通道数据做水深解算
Input:
Output:			CH1,CH2orCH3通道有效数据水深解算结果
*************************************************/
void ReadFile::readDeepByRed()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("ReadDeepByRedProcessing:");


	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream output_stream;
	output_stream.open("DeepByRedOut.txt", ios::out);

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

			//获取近红外水面点
			dw.GetRedTime(dw.m_RedDeep, dw.redTime);

			//process
			blueStd = calculateSigma(dw.m_BlueDeep);
			greenStd = calculateSigma(dw.m_GreenDeep);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

			switch (bgflag)
			{
			case BLUE:
				DeepWave::ostreamFlag = BLUE;

				dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_BlueDeepPra, dw.redTime, dw.blueDeepDepth);
				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_GreenDeepPra, dw.redTime, dw.greenDeepDepth);
				break;
			default:
				break;
			}

			//输出信息到文件
			output_stream << dw;

			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		printf("Finished!\n");
	}
}


/*************************************************
Function:       深水数据CH1，CH2CH3通道混合处理
Description:	近红外通道确定水面，蓝绿通道的标准差大小决定选择相应的通道数据做水深解算
Input:
Output:			输出点云格式X,Y,Z
*************************************************/
void ReadFile::readDeepOutLas()
{
	unsigned _int64 j = 0;
	HS_Lidar hs;

	//把文件的位置指针移到文件尾获取文件长度
	unsigned _int64 length;
	_fseeki64(m_filePtr, 0L, SEEK_END);
	length = _ftelli64(m_filePtr);
	printf("ReadDeepOutLasProcessing:");


	//首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
	fstream las_stream;
	las_stream.open("las2txt.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//有效水深的计数器，平均水深
	int count = 0;
	float avedepth = 0;
	double tmpX = 0.0;
	double tmpY = 0.0;

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

			//当经纬度发生变化时计算该经纬度的平均有效水深进行输出
			if ((!isEqual(hs.header.dX,tmpX) || !isEqual(hs.header.dY, tmpY))&&(count>0))
			{
				tmpX = hs.header.dX;
				tmpY = hs.header.dY;
				//控制输出精度
				las_stream << setiosflags(ios::fixed) << setiosflags(ios::showpoint) << setprecision(6) << tmpX << " " << tmpY << " " << setprecision(3) << avedepth / count << endl;
				
			}

			//获取近红外水面点
			dw.GetRedTime(dw.m_RedDeep, dw.redTime);

			//process
			blueStd = calculateSigma(dw.m_BlueDeep);
			greenStd = calculateSigma(dw.m_GreenDeep);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

			switch (bgflag)
			{
			case BLUE:
				DeepWave::ostreamFlag = BLUE;

				dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_BlueDeepPra, dw.redTime, dw.blueDeepDepth);

				//有效水深计数加一并求和
				if (dw.blueDeepDepth != 0)
				{
					avedepth += dw.blueDeepDepth;
					count++;
				}

				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_GreenDeepPra, dw.redTime, dw.greenDeepDepth);

				//有效水深计数加一并求和
				if (dw.greenDeepDepth != 0)
				{
					avedepth += dw.blueDeepDepth;
					count++;
				}

				break;
			default:
				break;
			}

		
			//文件指针偏移一帧完整数据的字节数：2688/8
			j += 336;

			//打印处理进程情况设置宽度输出精度
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

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
		las_stream.close();
		printf("Finished!\n");
	}
}