#pragma once

#include <iostream>
#include "WaveData.h"
using namespace std;

class ReadFile
{
public:
	ReadFile();
	~ReadFile();
	bool setFilename(char filename[100]);
	void readBlueAll();
	void readGreenAll();
	void readMix();
private:
	char *m_filename;
	FILE *m_filePtr;
};