// myLidar.cpp : 定义控制台应用程序的入口点。
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
			cout << "Channel?(0:Blue/1:Green/2:All/3:Mix):" << endl;
			cin >> flag;
			switch(flag)
			{
			case 0: {
				myfile.readBlueAll();
				break;
			}
			case 1:{
				myfile.readGreenAll();
				break;
			}
			case 2: {
				myfile.readBlueAll();
				myfile.readGreenAll();
				break;
			}
			case 3: {
				myfile.readMix();
				break;
			}
			}
		}
		cout << "continue?(1:Y/0:N):" << endl;
		cin >> flag;
	}
	
	return 0;
}