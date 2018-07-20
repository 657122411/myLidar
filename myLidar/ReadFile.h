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
	void readAll();
	void readByTime();
private:
	char *m_filename;
	FILE *m_filePtr;
};