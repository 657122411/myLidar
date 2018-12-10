#ifndef HS_Lidar_H
#define HS_Lidar_H

#include "HS_Lidar_Channel.h"
#include "HS_Lidar_Header.h"
#include "iostream"
#include "fstream"
#include <vector>
using namespace std;

class HS_Lidar
{
public:

	HS_Lidar_Header header;
	HS_Lidar_Channel CH1;
	HS_Lidar_Channel CH2;
	HS_Lidar_Channel CH3;
	HS_Lidar_Channel CH4;

	HS_Lidar();
	~HS_Lidar();

	void initData(FILE *fp);						//数据初始化
	void getHeader(FILE *fp);						//获取帧头
	void getChannel(FILE *fp, HS_Lidar_Channel &CH);//获取通道

	void initDeepData(FILE *fp);					//针对深水数据的初始化
	void getDeepChannel(FILE *fp, HS_Lidar_Channel &CH, vector<int> &deepData);//获得深水数据的通道数据

	vector<int> deepData1;
	vector<int> deepData2;
	vector<int> deepData3;
	vector<int> deepData4;
 	
};

#endif


