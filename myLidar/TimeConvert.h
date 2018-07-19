#ifndef TIMECONVERT_H
#define TIMECONVERT_H


typedef struct tagCOMMONTIME
{
	int   year;
	int   month;
	int   day;
	int   hour;
	int   minute;
	double   second;
}COMMONTIME;//ͨ��ʱ

typedef COMMONTIME *PCOMMONTIME;

typedef struct tagTOD
{
	long sn;  //��������������
	double tos;//������С������
}TOD;

typedef TOD *PTOD;

typedef struct
{
	long day; //��������
	TOD tod;  //һ���ڵ�����
}JULIANDAY;//������

typedef JULIANDAY *PJULIANDAY;

typedef struct tagMJULIANDAY
{
	long day;
	TOD  tod;
}MJULIANDAY;//��������

typedef MJULIANDAY *PMJIANDAY;

typedef struct tagTOW
{
	long sn;//����������
	double tos;//��С������
}TOW;

typedef TOW *PTOW;

typedef struct tagGPSTIME
{
	int wn; //����
	TOW tow;//һ���ڵ�����
}GPSTIME;//GPSʱ

typedef GPSTIME *PGPSTIME;

typedef struct tagDOY
{
	unsigned short year;
	unsigned short day;
	TOD tod;
}DOY;//�����

typedef DOY *PDOY;

double FRAC(double morigin);


void CommonTimeToJulianDay(PCOMMONTIME pct, PJULIANDAY pjd);

void JulianDayToCommonTime(PJULIANDAY pjd, PCOMMONTIME pct);

void JulianDayToGPSTime(PJULIANDAY pjd, PGPSTIME pgt);

void GPSTimeToJulianDay(PGPSTIME pgt, PJULIANDAY pjd);

void CommonTimeToGPSTime(PCOMMONTIME pct, PGPSTIME pgt);

void GPSTimeToCommonTime(PGPSTIME pgt, PCOMMONTIME pct);

void CommonTimeToDOY(PCOMMONTIME pct, PDOY pdoy);

void DOYToCommonTime(PDOY pdoy, PCOMMONTIME pct);

void GPSTimeToDOY(PGPSTIME pgt, PDOY pdoy);

void DOYToGPSTime(PDOY pdoy, PGPSTIME pgt);

void JulianDayToDOY(PJULIANDAY pjd, PDOY pdoy);

void DOYToJulianDay(PDOY pdoy, PJULIANDAY pjd);


#endif