/*
 *  2019.10.01 created by Kelly @nepes
 *  UTIL FOR TIMEER
 *  TimeUtil.hpp
 *  Copyright (c) 2019, nepes inc, All rights reserved.
 *
 */
#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <stdarg.h>
#include <time.h>
#include <chrono>

typedef struct _TIMER {
    unsigned int expire;
    void* routine;
    void* arg;
    short timer_id;
    clock_t last_time;
    bool last_check = 0;
    
} TIMER;


void InitTimer(TIMER *times);
void SetTimer(TIMER *times, unsigned int expire, void (*func)(void *), void *arg, int id);
long long GetTime();
void PassTime();

void StartTimer(TIMER *times);
int EndTimer(TIMER *times);
float LapTimer(TIMER *times);

std::string getSerial();
std::string getCurrentDateTime( std::string s );
void Logger(const char *format, ...);
