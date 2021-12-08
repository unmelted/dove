/*
 *  2019.10.01 created by Kelly @nepes
 *  UTIL FOR TIMER
 *  TimeUtil.cpp
 *  Copyright (c) 2019, nepes inc, All rights reserved.
 *
 */
#define _CRT_SECURE_NO_WARNINGS
#include "TimeUtil.hpp"

using namespace std;

void InitTimer(TIMER *times)
{
    times->expire = 0;
    times->routine = NULL;
    times->arg = 0;
    times->timer_id = 0;
    times->last_check = 0;
}

void SetTimer(TIMER *times, unsigned int expire, void (*func)(void *), void *arg, int id)
{
    times->expire = expire;
//    times->routine = func;
    times->arg = arg;
    times->timer_id = id;
    return;
}

void StartTimer(TIMER *times)
{
    times->last_check = true;
    times->last_time = clock();
}

int EndTimer(TIMER *times)
{
    if(times->last_check == 0) return -1;
    int ms = clock() - times->last_time;
    times->last_check = 0;
    return ms;
}

float LapTimer(TIMER *times)
{
    if(times->last_check == 0) return -1;
    clock_t ctime = clock();
    float ms = ((float)ctime - (float)times->last_time)/1000.0;
    times->last_time = ctime;
    times->last_check = true;
    return ms;
}

string getCurrentDateTime( string s )
{
    time_t now = time(0);
    struct tm  tstruct;
    char  buf[80];
    tstruct = *localtime(&now);
    if(s=="now")
        strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
    else if(s=="date")
        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return string(buf);
}

long long GetTime()
{
    auto time = std::chrono::system_clock::now();
    auto mill = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch());

    long long currentTimeMillis = mill.count();
    return currentTimeMillis;
}

void PassTime()
{
    auto time = std::chrono::system_clock::now();
    auto mill = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch());

    long long currentTimeMillis = mill.count();
    int msc = currentTimeMillis % 1000;
    long nowTime = currentTimeMillis / 1000;

    tm *ts = localtime((const time_t *)&nowTime);

    int year = ts->tm_year + 1900;
    int month = ts->tm_mon + 1;
    int day = ts->tm_mday;
    int hour = ts->tm_hour;
    int min = ts->tm_min;
    int sec = ts->tm_sec;
    char temp[1024] = { '\0' };
    printf("%04d-%02d-%02d %02d:%02d:%02d.%03d   \n",  year, month, day, hour, min, sec, msc);

}

string getSerial()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%m%d%H%M%S", timeinfo);
	std::string dstr(buffer); 

	return dstr;
}

// void Stopper()
// {
//     string filePath = "log/stopper.txt";
//     ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app );
//     ofs << "stop"  << '\t' << '\n';
//     ofs.close();
//     flag_file = 1;
// }

void Logger(const char *format, ...)
{
#if defined _WIN_ || _WINDOWS
    string filePath = "recalibration\\log\\logd_"+getCurrentDateTime("date")+".txt";
#else
    string filePath = "recalibration/log/logd_"+getCurrentDateTime("date")+".txt";
#endif    
    va_list ap;
    char buf[4096];
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);

    string now = getCurrentDateTime("now");
    ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app );
    ofs << now << '\t' << buf << endl;

#ifdef _DEBUG
     printf("[%s]\t%s \n", now.c_str(), buf); fflush(stdout);
#endif

    ofs.close();
}

void Dlog::Logger(const char* format, ...) {

#if defined _WIN_ || _WINDOWS
    string filePath = "recalibration\\log\\log_" + fname +"_" + getCurrentDateTime("date") + ".txt";
#else
    string filePath = "recalibration/log/logd_" + fname +"_" + getCurrentDateTime("date") + ".txt";
#endif    
    va_list ap;
    char buf[4096];
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);

    string now = getCurrentDateTime("now");
    ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app);
    ofs << now << '\t' << buf << endl;

#ifdef _DEBUG
    printf("[%s]\t%s\t%s \n", now.c_str(), fname.c_str(), buf); fflush(stdout);
#endif

    ofs.close();
}