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

	void initData(FILE *fp);						//���ݳ�ʼ��
	void getHeader(FILE *fp);						//��ȡ֡ͷ
	void getChannel(FILE *fp, HS_Lidar_Channel &CH);//��ȡͨ��

	void initDeepData(FILE *fp);					//�����ˮ���ݵĳ�ʼ��
	void getDeepChannel(FILE *fp, HS_Lidar_Channel &CH, vector<int> &deepData);//�����ˮ���ݵ�ͨ������

	vector<int> deepData1;
	vector<int> deepData2;
	vector<int> deepData3;
	vector<int> deepData4;
 	
};

#endif


