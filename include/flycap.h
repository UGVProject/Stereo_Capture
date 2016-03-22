#include "FlyCapture2.h"
#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <dirent.h>
#ifdef LINUX
#include <unistd.h>
#endif


#define UTC (0)
#define shuttlespeed 2

extern unsigned int capNum;
//global variables
//extern int display_counter;
extern int numberofoutsync;
extern char datestr[20];
extern char timestr[20];
extern time_t currentdate;
extern time_t now;
extern struct tm *timenow;
extern char finalarray[50];
extern char folderstring[14];
extern char txtfile[50];
extern unsigned int numCameras;
extern int numImages;
extern double iterTime;
extern double Sumiter;
extern int currentindex;
extern char filename0[50];
extern char filename1[50];
extern int fileindex;
extern char input;
extern int mode;
extern char extent;
extern char* extent3;


static int help();

double GetTickCount(void) ;

//void PrintBuildInfo();

void PrintCameraInfo_l( FlyCapture2::CameraInfo* pCamInfo );

void PrintCameraInfo_r( FlyCapture2::CameraInfo* pCamInfo );

void PrintProperty_l( FlyCapture2::Property* pProp);

void PrintProperty_r( FlyCapture2::Property* pProp);

void PrintError( FlyCapture2::Error error );

bool PollForTriggerReady( FlyCapture2::Camera* pCam );	//This is just for software trigger

bool CreateDirectory(char *datechar, char *timechar);