// myLidar.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "WaveData.h"
#include "ReadFile.h"
using namespace std;

int main()
{
	int flag = 1;
	while (flag)
	{
		cout << "file address:"<<endl;
		char name[100];
		cin >> name;
		ReadFile myfile;
		bool ret = myfile.setFilename(name);
		if (ret)
		{
			myfile.readAll();
		}
		cout << "continue?(1/0)" << endl;
		cin >> flag;
	}
	
	return 0;
}