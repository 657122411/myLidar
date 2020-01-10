// myLidar.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include "stdio.h"
#include "ReadFile.h"
using namespace std;

float angle = 0;

int main()
{
	int flag = 1;
	while (flag)
	{
		printf("Enter the absolute path to the file to be processed:\n");
		char name[100];
		scanf("%s", name);
		ReadFile myfile;
		bool ret = myfile.setFilename(name);
		if (ret)
		{
			printf("\nSelect the channel to be processed?\n");
			printf("===================================\n");
			printf("0:Blue.\n");
			printf("1:Green.\n");
			printf("2:All.\n");
			printf("3:Mix.\n");
			printf("4:OutputData.\n");
			printf("5:ReadDeep.\n");
			printf("6:ReadDeepByRed.\n");
			printf("7:ReadDeepOutLas.\n");
			printf("===================================\n\n");
			scanf("%d", &flag);
			switch (flag)
			{
			case 0: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readBlueAll();
				break;
			}
			case 1: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readGreenAll();
				break;
			}
			case 2: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readBlueAll();
				myfile.readGreenAll();
				break;
			}
			case 3: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readMix();
				break;
			}
			case 4: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.outputData();
				break;
			}
			case 5: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readDeep();
				break;
			}
			case 6: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readDeepByRed();
				break;
			}
			case 7: {
				printf("\nLaser incidence angle(°)?\n");
				scanf("%f", &angle);
				myfile.readDeepOutLas();
				break;
			}
			}
		}
		else
			continue;

		printf("\nContinue?(1:Y/0:N):\n\n");
		scanf("%d", &flag);
	}
	
	return 0;
}