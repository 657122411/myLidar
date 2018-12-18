#include "DeepWave.h"
#include <numeric>
#include <algorithm>

#define DeepPulseWidth 4		//定义激光脉冲宽度做剥离阈值参考
#define TimeDifference 8	//与UTC的时差

#define BLUE true
#define GREEN false

#define c 0.3				//相对光速乘以纳秒
#define ndeepwater 1.34		//海水水质的折射率

// 回波类型
#define DEEPSURFACE true		//水表回波（可能包括后向散射）
#define DEEPBOTTOM false		//水底或水中物质回波

bool DeepWave::ostreamFlag = BLUE;


//五点线性平滑
void linearSmooth5(float in[], float out[], int N)
{
	int i;
	if (N < 5)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (3.0 * in[0] + 2.0 * in[1] + in[2] - in[4]) / 5.0;
		out[1] = (4.0 * in[0] + 3.0 * in[1] + 2 * in[2] + in[3]) / 10.0;
		for (i = 2; i <= N - 3; i++)
		{
			out[i] = (in[i - 2] + in[i - 1] + in[i] + in[i + 1] + in[i + 2]) / 5.0;
		}
		out[N - 2] = (4.0 * in[N - 1] + 3.0 * in[N - 2] + 2 * in[N - 3] + in[N - 4]) / 10.0;
		out[N - 1] = (3.0 * in[N - 1] + 2.0 * in[N - 2] + in[N - 3] - in[N - 5]) / 5.0;
	}
}


//七点线性平滑
void linearSmooth7(float in[], float out[], int N)
{
	int i;
	if (N < 7)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (13.0 * in[0] + 10.0 * in[1] + 7.0 * in[2] + 4.0 * in[3] +
			in[4] - 2.0 * in[5] - 5.0 * in[6]) / 28.0;

		out[1] = (5.0 * in[0] + 4.0 * in[1] + 3 * in[2] + 2 * in[3] +
			in[4] - in[6]) / 14.0;

		out[2] = (7.0 * in[0] + 6.0 * in[1] + 5.0 * in[2] + 4.0 * in[3] +
			3.0 * in[4] + 2.0 * in[5] + in[6]) / 28.0;

		for (i = 3; i <= N - 4; i++)
		{
			out[i] = (in[i - 3] + in[i - 2] + in[i - 1] + in[i] + in[i + 1] + in[i + 2] + in[i + 3]) / 7.0;
		}

		out[N - 3] = (7.0 * in[N - 1] + 6.0 * in[N - 2] + 5.0 * in[N - 3] +
			4.0 * in[N - 4] + 3.0 * in[N - 5] + 2.0 * in[N - 6] + in[N - 7]) / 28.0;

		out[N - 2] = (5.0 * in[N - 1] + 4.0 * in[N - 2] + 3.0 * in[N - 3] +
			2.0 * in[N - 4] + in[N - 5] - in[N - 7]) / 14.0;

		out[N - 1] = (13.0 * in[N - 1] + 10.0 * in[N - 2] + 7.0 * in[N - 3] +
			4 * in[N - 4] + in[N - 5] - 2 * in[N - 6] - 5 * in[N - 7]) / 28.0;
	}
}


//五点二次函数拟合平滑
void quadraticSmooth5(float in[], float out[], int N)
{
	int i;
	if (N < 5)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (31.0 * in[0] + 9.0 * in[1] - 3.0 * in[2] - 5.0 * in[3] + 3.0 * in[4]) / 35.0;
		out[1] = (9.0 * in[0] + 13.0 * in[1] + 12 * in[2] + 6.0 * in[3] - 5.0 *in[4]) / 35.0;
		for (i = 2; i <= N - 3; i++)
		{
			out[i] = (-3.0 * (in[i - 2] + in[i + 2]) +
				12.0 * (in[i - 1] + in[i + 1]) + 17 * in[i]) / 35.0;
		}
		out[N - 2] = (9.0 * in[N - 1] + 13.0 * in[N - 2] + 12.0 * in[N - 3] + 6.0 * in[N - 4] - 5.0 * in[N - 5]) / 35.0;
		out[N - 1] = (31.0 * in[N - 1] + 9.0 * in[N - 2] - 3.0 * in[N - 3] - 5.0 * in[N - 4] + 3.0 * in[N - 5]) / 35.0;
	}
}


//七点二次函数拟合平滑
void quadraticSmooth7(float in[], float out[], int N)
{
	int i;
	if (N < 7)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (32.0 * in[0] + 15.0 * in[1] + 3.0 * in[2] - 4.0 * in[3] -
			6.0 * in[4] - 3.0 * in[5] + 5.0 * in[6]) / 42.0;

		out[1] = (5.0 * in[0] + 4.0 * in[1] + 3.0 * in[2] + 2.0 * in[3] +
			in[4] - in[6]) / 14.0;

		out[2] = (1.0 * in[0] + 3.0 * in[1] + 4.0 * in[2] + 4.0 * in[3] +
			3.0 * in[4] + 1.0 * in[5] - 2.0 * in[6]) / 14.0;
		for (i = 3; i <= N - 4; i++)
		{
			out[i] = (-2.0 * (in[i - 3] + in[i + 3]) +
				3.0 * (in[i - 2] + in[i + 2]) +
				6.0 * (in[i - 1] + in[i + 1]) + 7.0 * in[i]) / 21.0;
		}
		out[N - 3] = (1.0 * in[N - 1] + 3.0 * in[N - 2] + 4.0 * in[N - 3] +
			4.0 * in[N - 4] + 3.0 * in[N - 5] + 1.0 * in[N - 6] - 2.0 * in[N - 7]) / 14.0;

		out[N - 2] = (5.0 * in[N - 1] + 4.0 * in[N - 2] + 3.0 * in[N - 3] +
			2.0 * in[N - 4] + in[N - 5] - in[N - 7]) / 14.0;

		out[N - 1] = (32.0 * in[N - 1] + 15.0 * in[N - 2] + 3.0 * in[N - 3] -
			4.0 * in[N - 4] - 6.0 * in[N - 5] - 3.0 * in[N - 6] + 5.0 * in[N - 7]) / 42.0;
	}
}


/*
* 五点三次平滑
*/
void cubicSmooth5(float in[], float out[], int N)
{

	int i;
	if (N < 5)
	{
		for (i = 0; i <= N - 1; i++)
			out[i] = in[i];
	}

	else
	{
		out[0] = (69.0 * in[0] + 4.0 * in[1] - 6.0 * in[2] + 4.0 * in[3] - in[4]) / 70.0;
		out[1] = (2.0 * in[0] + 27.0 * in[1] + 12.0 * in[2] - 8.0 * in[3] + 2.0 * in[4]) / 35.0;
		for (i = 2; i <= N - 3; i++)
		{
			out[i] = (-3.0 * (in[i - 2] + in[i + 2]) + 12.0 * (in[i - 1] + in[i + 1]) + 17.0 * in[i]) / 35.0;
		}
		out[N - 2] = (2.0 * in[N - 5] - 8.0 * in[N - 4] + 12.0 * in[N - 3] + 27.0 * in[N - 2] + 2.0 * in[N - 1]) / 35.0;
		out[N - 1] = (-in[N - 5] + 4.0 * in[N - 4] - 6.0 * in[N - 3] + 4.0 * in[N - 2] + 69.0 * in[N - 1]) / 70.0;
	}
	return;
}


/*
* 七点三次平滑
*/
void cubicSmooth7(float in[], float out[], int N)
{
	int i;
	if (N < 7)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (39.0 * in[0] + 8.0 * in[1] - 4.0 * in[2] - 4.0 * in[3] +
			1.0 * in[4] + 4.0 * in[5] - 2.0 * in[6]) / 42.0;
		out[1] = (8.0 * in[0] + 19.0 * in[1] + 16.0 * in[2] + 6.0 * in[3] -
			4.0 * in[4] - 7.0* in[5] + 4.0 * in[6]) / 42.0;
		out[2] = (-4.0 * in[0] + 16.0 * in[1] + 19.0 * in[2] + 12.0 * in[3] +
			2.0 * in[4] - 4.0 * in[5] + 1.0 * in[6]) / 42.0;
		for (i = 3; i <= N - 4; i++)
		{
			out[i] = (-2.0 * (in[i - 3] + in[i + 3]) +
				3.0 * (in[i - 2] + in[i + 2]) +
				6.0 * (in[i - 1] + in[i + 1]) + 7.0 * in[i]) / 21.0;
		}
		out[N - 3] = (-4.0 * in[N - 1] + 16.0 * in[N - 2] + 19.0 * in[N - 3] +
			12.0 * in[N - 4] + 2.0 * in[N - 5] - 4.0 * in[N - 6] + 1.0 * in[N - 7]) / 42.0;
		out[N - 2] = (8.0 * in[N - 1] + 19.0 * in[N - 2] + 16.0 * in[N - 3] +
			6.0 * in[N - 4] - 4.0 * in[N - 5] - 7.0 * in[N - 6] + 4.0 * in[N - 7]) / 42.0;
		out[N - 1] = (39.0 * in[N - 1] + 8.0 * in[N - 2] - 4.0 * in[N - 3] -
			4.0 * in[N - 4] + 1.0 * in[N - 5] + 4.0 * in[N - 6] - 2.0 * in[N - 7]) / 42.0;
	}
}


/*功能：	计算数据的标准差
//*:
//resultSet：传入的数据数组
//stdev：	返回值为标准差
//*
*/
float calculateDeepSigma(vector<float> resultSet)
{
	double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
	double mean = sum / resultSet.size(); //均值  

	double accum = 0.0;
	for each (float d in resultSet)
	{
		accum += (d - mean)*(d - mean);
	}

	float stdev = sqrt(accum / (resultSet.size() - 1)); //方差  

	return stdev;
}


//峰值检测算法 https://www.mathworks.com/help/signal/ref/findpeaks.html
std::vector<int> FindLocalMaxima(std::vector<int> dataVector, int minProminence, int maxProminence, int minWidth, int maxWidth)
{
	int dataVectorSize = dataVector.size();
	std::vector<int>localMaximaIndex, localMaximaValue, prominenceValue, inRangeLocalMaxima;
	int j;
	int currentValue, nextValue, prevValue;
	int totalLocalMaxima;
	int prevLocalMaximaIndex;
	int nextLocalMaximaIndex;
	int prevHigherLocalMaximaValue;
	int nextHigherLocalMaximaValue;
	int leftWidth, rightWidth, width;
	int prevMinimum, nextMinimum, minimumValue;
	int prominence;
	int i = 1;
	/********************* FIND LOCAL MAXIMA ***************************/
	while (i<dataVectorSize-1)
	{
		j = 1;
		currentValue = dataVector[i];
		prevValue = dataVector[i - 1];
		nextValue = dataVector[i + j];

		if ((prevValue<currentValue) && (currentValue >= nextValue))
		{
			while ((currentValue == nextValue) && ((i + j)<dataVectorSize-1))
			{
				j++;
				nextValue = dataVector[i + j];
			}
			if (currentValue>nextValue)
			{
				localMaximaIndex.push_back(i);
				localMaximaValue.push_back(currentValue);
			}
		}
		i += j;
	}

	totalLocalMaxima = localMaximaIndex.size();
	prominenceValue.erase(prominenceValue.begin(), prominenceValue.end());

	/********************* GET PROMINENCE ***************************/
	for (i = 0; i<totalLocalMaxima; i++)
	{
		prevLocalMaximaIndex = i - 1;
		nextLocalMaximaIndex = i + 1;
		prevHigherLocalMaximaValue = 0;
		nextHigherLocalMaximaValue = 0;

		if (prevLocalMaximaIndex >= 0)
		{
			while ((prevLocalMaximaIndex >= 1) && (localMaximaValue[prevLocalMaximaIndex] <= localMaximaValue[i]))
			{
				prevLocalMaximaIndex--;
			}
			if (localMaximaValue[prevLocalMaximaIndex]>localMaximaValue[i]) {
				prevHigherLocalMaximaValue = localMaximaValue[prevLocalMaximaIndex];
			}
			else {
				prevHigherLocalMaximaValue = 0;
			}
		}
		if (nextLocalMaximaIndex<totalLocalMaxima)
		{
			while ((nextLocalMaximaIndex<totalLocalMaxima - 1) && (localMaximaValue[nextLocalMaximaIndex] <= localMaximaValue[i]))//
			{
				nextLocalMaximaIndex++;
			}
			if (localMaximaValue[nextLocalMaximaIndex]>localMaximaValue[i]) {
				nextHigherLocalMaximaValue = localMaximaValue[nextLocalMaximaIndex];
			}
			else {
				nextHigherLocalMaximaValue = 0;
			}
		}
		prevMinimum = dataVector[localMaximaIndex[i]];
		nextMinimum = dataVector[localMaximaIndex[i]];
		minimumValue = dataVector[localMaximaIndex[i]];
		if ((prevHigherLocalMaximaValue == 0) && (nextHigherLocalMaximaValue == 0))
		{
			for (j = 0; j<dataVectorSize; j++)
			{
				if (dataVector[j]<minimumValue)
					minimumValue = dataVector[j];
			}


		}
		else
		{
			if (prevLocalMaximaIndex<0)
			{
				for (j = localMaximaIndex[i]; j >= 0; j--)
				{
					if (dataVector[j]<prevMinimum)
						prevMinimum = dataVector[j];
				}

			}
			else
			{
				for (j = localMaximaIndex[i]; j>localMaximaValue[prevLocalMaximaIndex]; j--)
				{
					if (dataVector[j]<prevMinimum)
						prevMinimum = dataVector[j];
				}

			}
			if (nextLocalMaximaIndex >= totalLocalMaxima - 1)
			{
				for (j = localMaximaIndex[i]; j<dataVectorSize; j++)
				{
					if (dataVector[j]<nextMinimum)
						nextMinimum = dataVector[j];
				}

			}
			else
			{
				for (j = localMaximaIndex[i]; j<localMaximaIndex[nextLocalMaximaIndex]; j++)
				{
					if (dataVector[j]<nextMinimum)
						nextMinimum = dataVector[j];
				}

			}
			minimumValue = prevMinimum>nextMinimum ? prevMinimum : nextMinimum;
		}
		prominence = localMaximaValue[i] - minimumValue;
		/*************************** GET WIDTH ********************************/
		j = 1;
		while (((localMaximaIndex[i] - j)>0) && (dataVector[localMaximaIndex[i] - j] - minimumValue>(prominence) / 2)) { j++; }
		leftWidth = j;
		j = 1;
		while (((localMaximaIndex[i] + j)<dataVectorSize) && (dataVector[localMaximaIndex[i] + j] - minimumValue>(prominence) / 2)) { j++; }
		rightWidth = j;
		int width = leftWidth + rightWidth;

		/********************* IS THE PEAK IN RANGE? ***************************/
		if ((localMaximaValue[i] - minimumValue >= minProminence) && (localMaximaValue[i] - minimumValue <= maxProminence) && (width >= minWidth) && (width <= maxWidth))
		{
			inRangeLocalMaxima.push_back(localMaximaIndex[i]);
		}

	}
	cout << 1;
	return inRangeLocalMaxima;
	
}


/*功能：	假设两组高斯函数模型
//*p:	代求参数
//*x：  原始数据（测量值）
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void deep_expfun2(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//写出参数与x[i]之间的关系式，由于这里方程的右边没有观测值，所以只有参数
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]));
	}
}


/*功能：	两组高斯函数模型的雅可比矩阵
//*p:	代求参数
//jac： 雅可比矩阵参数
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void deep_jacexpfun2(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//写出雅克比矩阵
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
	}
}


/*功能：	假设三组高斯函数模型
//*p:	代求参数
//*x：  原始数据（测量值）
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void deep_expfun3(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//写出参数与x[i]之间的关系式，由于这里方程的右边没有观测值，所以只有参数
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]))
			+ p[6] * exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*(2 * p[8]));
	}
}


/*功能：	三组高斯函数模型的雅可比矩阵
//*p:	代求参数
//jac： 雅可比矩阵参数
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void deep_jacexpfun3(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//写出雅克比矩阵
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));

		jac[j++] = exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*p[8]);
		jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));
		jac[j++] = p[6] * (i - p[7])*(i - p[7]) / (p[8] * p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));
	}
}


/*功能：	假设四组高斯函数模型
//*p:	代求参数
//*x：  原始数据（测量值）
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void deep_expfun4(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//写出参数与x[i]之间的关系式，由于这里方程的右边没有观测值，所以只有参数
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]))
			+ p[6] * exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*(2 * p[8]))
			+ p[9] * exp(-(i - p[10])*(i - p[10]) / (2 * p[11])*(2 * p[11]));
	}

}


/*功能：	四组高斯函数模型的雅可比矩阵
//*p:	代求参数
//jac： 雅可比矩阵参数
//m：	参数维度
//n：	测量值维度
//*data:？
*/
void deep_jacexpfun4(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//写出雅克比矩阵
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));

		jac[j++] = exp(-(i - p[7])*(i - p[7]) / (2 * p[8])*p[8]);
		jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));
		jac[j++] = p[6] * (i - p[7])*(i - p[7]) / (p[8] * p[8] * p[8])*exp(-(i - p[7])*(i - p[7]) / (2 * p[8] * p[8]));

		jac[j++] = exp(-(i - p[10])*(i - p[10]) / (2 * p[11])*p[11]);
		jac[j++] = p[9] * (i - p[10]) / (p[11] * p[11])*exp(-(i - p[10])*(i - p[10]) / (2 * p[11] * p[11]));
		jac[j++] = p[9] * (i - p[10])*(i - p[10]) / (p[11] * p[11] * p[11])*exp(-(i - p[10])*(i - p[10]) / (2 * p[11] * p[11]));
	}
}


/*
//功能：构造函数初始化数据
*/
DeepWave::DeepWave()
{
	m_time = { 0,0,0,0,0,0 };
	m_BlueDeepNoise = 0;
	m_GreenDeepNoise = 0;
	blueDeepDepth = 0;
	greenDeepDepth = 0;
}


DeepWave::~DeepWave()
{
}


/*功能：		获取原始数据中深浅水通道的二段回波数据
//&hs:		通道原始数据
*/
void DeepWave::GetDeepData(HS_Lidar & hs)
{
	//GPS->UTC->BeiJing
	PGPSTIME pgt = new GPSTIME;
	PCOMMONTIME pct = new COMMONTIME;
	pgt->wn = (int)hs.header.nGPSWeek;
	pgt->tow.sn = (long)hs.header.dGPSSecond;
	pgt->tow.tos = 0;
	GPSTimeToCommonTime(pgt, pct);
	m_time.year = pct->year;
	m_time.month = pct->month;
	m_time.day = pct->day;
	m_time.hour = pct->hour + TimeDifference;	//直接转化为北京时间
	m_time.minute = pct->minute;
	m_time.second = pct->second;
	delete pgt;
	delete pct;

	//取蓝绿通道深水数据
	vector<int >::iterator it;//声明迭代器
	for (it = hs.deepData2.begin(); it != hs.deepData2.end(); ++it) 
	{
		m_BlueDeep.push_back((float)*it);
	}
	for (it = hs.deepData3.begin(); it != hs.deepData3.end(); ++it) 
	{
		m_GreenDeep.push_back((float)*it);
	}
}


/*功能：		预处理数据：截取有效部分并进行去噪滤波操作
//			深水采用SG滤波器 https://blog.csdn.net/liyuanbhu/article/details/11119081
//&srcWave:	通道原始数据
//&noise：	记录的噪声所属波段
*/
void DeepWave::DeepFilter(vector<float> &srcWave, float &noise)
{
	//高斯滤波去噪
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	
	//申请数组内存
	float *buffer = new float[dstWave.size()];
	if (!dstWave.empty()) 
	{ 
		memcpy(buffer, &dstWave[0], dstWave.size() * sizeof(float));
	}
	float *buffer1 = new float[dstWave.size()];
	float *buffer2 = new float[dstWave.size()];
	float *buffer3 = new float[dstWave.size()];

	//滤波操作
	linearSmooth5(buffer, buffer1, dstWave.size());
	quadraticSmooth5(buffer1, buffer2, dstWave.size());
	cubicSmooth5(buffer2, buffer3, dstWave.size());

	dstWave.assign(&buffer3[0], &buffer3[dstWave.size()]);


	//释放内存
	delete buffer; 
	buffer = nullptr;
	delete buffer1; 
	buffer1 = nullptr;
	delete buffer2; 
	buffer2 = nullptr;
	delete buffer3; 
	buffer3 = nullptr;

	noise = 0;
	//计算随机噪声:两次滤波前后的波形数据的峰值差的均方差（标准差）
	for (int i = 0; i < srcWave.size(); i++)
	{
		noise += (srcWave.at(i) - dstWave.at(i)) * (srcWave.at(i) - dstWave.at(i));
	}
	noise = sqrt(noise / srcWave.size());

	srcWave.assign(dstWave.begin(), dstWave.end());
}


/*功能：			高斯分量分解函数
//&srcWave:		通道原始数据
//&waveParam：	该通道的高斯分量参数
*/
void DeepWave::DeepResolve(vector<float> &srcWave, vector<float> &waveParam, float &noise)
{
	//拷贝原始数据
	vector<float> data,temp;
	data.assign(srcWave.begin(), srcWave.end());

	//将滤波后的数据最小值作为背景噪声
	vector<float>::iterator smallest = min_element(begin(data), end(data));
	float backgroundNoise = *smallest;

	//所有数据除去环境噪声
	for (vector<float>::iterator m = data.begin(); m != data.end(); m++) //用迭代器的方式
	{
		*m -= backgroundNoise;
	}

	vector<int> intData;
	for (auto d : data)
	{
		intData.push_back((int)d);
	}

	//寻找峰值
	FindLocalMaxima(intData, 10, 900, 4, 50);

	//for (auto ans : answer)
	//{
	//	waveParam.push_back((float)ans);
	//}
}


///*功能：			LM算法迭代优化
////&srcWave:		通道原始数据
////&waveParam：	该通道的高斯分量参数
////LM算法参考：	https://blog.csdn.net/shajun0153/article/details/75073137
//*/
//void DeepWave::DeepOptimize(vector<float> &srcWave, vector<float> &waveParam)
//{
//	//解算初值为双峰
//	if (waveParam.size() == 2)
//	{
//		//获取高斯函数参数
//		double p[6];
//		int i = 0;
//		for (auto gp : waveParam)
//		{
//			p[i++] = gp.DA;
//			p[i++] = gp.Db;
//			p[i++] = gp.Dsigma;
//		}
//		int m = i;
//		int n = srcWave.size();
//
//		//获取拟合数据
//		double x[800];
//		i = 0;
//		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
//		{
//			x[i] = *iter;
//		}
//
//		double info[LM_INFO_SZ];
//		// 调用迭代入口函数
//		int ret = dlevmar_der(deep_expfun2,	//描述测量值之间关系的函数指针
//			deep_jacexpfun2,					//估计雅克比矩阵的函数指针
//			p,							//初始化的待求参数，结果一并保存在其中
//			x,							//测量值
//			m,							//参数维度
//			n,							//测量值维度
//			1000,						//最大迭代次数
//			NULL,						//opts,       //迭代的一些参数
//			info,						//关于最小化结果的一些参数，不需要设为NULL
//			NULL, NULL, NULL			//一些内存的指针，暂时不需要
//		);
//		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
//		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
//		printf("波峰时间差: %.7g ns\n", abs(p[4] - p[1]));*/
//
//		//将优化后的参数组赋给vector
//		i = 0;
//		for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
//		{
//			DeepgaussPraIter->DA = p[i++];
//			DeepgaussPraIter->Db = p[i++];
//			DeepgaussPraIter->Dsigma = p[i++];
//		}
//	}
//
//	//解算初值为三个峰
//	else if (waveParam.size() == 3)
//	{
//		//获取高斯函数参数
//		double p[9];
//		int i = 0;
//		for (auto gp : waveParam)
//		{
//			p[i++] = gp.DA;
//			p[i++] = gp.Db;
//			p[i++] = gp.Dsigma;
//		}
//		int m = i;
//		int n = srcWave.size();
//
//		//获取拟合数据
//		double x[800];
//		i = 0;
//		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
//		{
//			x[i] = *iter;
//		}
//
//		double info[LM_INFO_SZ];
//		// 调用迭代入口函数
//		int ret = dlevmar_der(deep_expfun3,	//描述测量值之间关系的函数指针
//			deep_jacexpfun3,					//估计雅克比矩阵的函数指针
//			p,							//初始化的待求参数，结果一并保存在其中
//			x,							//测量值
//			m,							//参数维度
//			n,							//测量值维度
//			1000,						//最大迭代次数
//			NULL,						//opts,       //迭代的一些参数
//			info,						//关于最小化结果的一些参数，不需要设为NULL
//			NULL, NULL, NULL			//一些内存的指针，暂时不需要
//		);
//		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
//		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
//		printf("波峰时间差: %.7g ns\n", abs(p[4] - p[1]));*/
//
//		//将优化后的参数组赋给vector
//		i = 0;
//		for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
//		{
//			DeepgaussPraIter->DA = p[i++];
//			DeepgaussPraIter->Db = p[i++];
//			DeepgaussPraIter->Dsigma = p[i++];
//		}
//	}
//
//	//解算初值为四个个峰
//	else if (waveParam.size() == 4)
//	{
//		//获取高斯函数参数
//		double p[12];
//		int i = 0;
//		for (auto gp : waveParam)
//		{
//			p[i++] = gp.DA;
//			p[i++] = gp.Db;
//			p[i++] = gp.Dsigma;
//		}
//		int m = i;
//		int n = srcWave.size();
//
//		//获取拟合数据
//		double x[800];
//		i = 0;
//		for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i)
//		{
//			x[i] = *iter;
//		}
//
//		double info[LM_INFO_SZ];
//		// 调用迭代入口函数
//		int ret = dlevmar_der(deep_expfun4,	//描述测量值之间关系的函数指针
//			deep_jacexpfun4,					//估计雅克比矩阵的函数指针
//			p,							//初始化的待求参数，结果一并保存在其中
//			x,							//测量值
//			m,							//参数维度
//			n,							//测量值维度
//			1000,						//最大迭代次数
//			NULL,						//opts,       //迭代的一些参数
//			info,						//关于最小化结果的一些参数，不需要设为NULL
//			NULL, NULL, NULL			//一些内存的指针，暂时不需要
//		);
//		/*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
//		printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
//		printf("波峰时间差: %.7g ns\n", abs(p[4] - p[1]));*/
//
//		//将优化后的参数组赋给vector
//		i = 0;
//		for (DeepgaussPraIter = waveParam.begin(); DeepgaussPraIter != waveParam.end(); DeepgaussPraIter++)
//		{
//			DeepgaussPraIter->DA = p[i++];
//			DeepgaussPraIter->Db = p[i++];
//			DeepgaussPraIter->Dsigma = p[i++];
//		}
//	}
//
//	return;
//}
//
//
/*功能：	计算水深
//内容：	提取波峰数目小于两个的直接剔除，否则取第一个（即能量最大值）为水面回波，脉冲时间最晚的为水底回波，计算水深
*/
void DeepWave::calculateDeepDepth(vector<float>& waveParam, float &BorGDepth)
{
	return ;
}


/*功能：	自定义需要输出的信息
//内容：	年 月 日 时 分 秒
*/
ostream &operator<<(ostream & stream, const DeepWave & wavedata)
{
	stream << wavedata.m_time.year << " "
		<< wavedata.m_time.month << " "
		<< wavedata.m_time.day << " "
		<< wavedata.m_time.hour << " "
		<< wavedata.m_time.minute << " "
		<< wavedata.m_time.second;

	//兴趣数据暂定为制定通道的波峰所在相对位置
	switch (wavedata.ostreamFlag)
	{
	case BLUE: {
		stream << " " << wavedata.blueDeepDepth << "m";

	
		break;
	}
	case GREEN: {
		stream << " " << wavedata.greenDeepDepth << "m";
		
		break;
	}
	}

	stream << endl;
	return stream;
}