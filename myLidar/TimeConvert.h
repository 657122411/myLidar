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
}COMMONTIME;//通用时

typedef COMMONTIME *PCOMMONTIME;

typedef struct tagTOD
{
	long sn;  //秒数的整数部分
	double tos;//秒数的小数部分
}TOD;

typedef TOD *PTOD;

typedef struct
{
	long day; //整数天数
	TOD tod;  //一天内的秒数
}JULIANDAY;//儒略日

typedef JULIANDAY *PJULIANDAY;

typedef struct tagMJULIANDAY
{
	long day;
	TOD  tod;
}MJULIANDAY;//新儒略日

typedef MJULIANDAY *PMJIANDAY;

typedef struct tagTOW
{
	long sn;//秒整数部分
	double tos;//秒小数部分
}TOW;

typedef TOW *PTOW;

typedef struct tagGPSTIME
{
	int wn; //周数
	TOW tow;//一周内的秒数
}GPSTIME;//GPS时

typedef GPSTIME *PGPSTIME;

typedef struct tagDOY
{
	unsigned short year;
	unsigned short day;
	TOD tod;
}DOY;//年积日

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