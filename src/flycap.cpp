//GreyPoint Library
#include "flycap.h"

using namespace std;
//int display_counter=0;
int numberofoutsync=0;
char datestr[20];
char timestr[20];
time_t currentdate = time(NULL);
time_t now;
struct tm *timenow;
char finalarray[50];
char folderstring[14];
char txtfile[50];
unsigned int capNum = 1000;
unsigned int numCameras;
int numImages = 0;
double iterTime = 0.0;
double Sumiter = 0.0;
int currentindex=0;
char filename0[50];
char filename1[50];
int fileindex=0;
char input=0;
int mode = 0;		//default mode is 0, calibration mode is 2 ;
char extent;
char* extent3 = "png";

double GetTickCount(void) 
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}

/*void PrintBuildInfo()
{
	FlyCapture2::FC2Version fc2Version;
	FlyCapture2::Utilities::GetLibraryVersion(&fc2Version);
	char version[128];
	sprintf( 
		version, 
		"FlyCapture2 library version: %d.%d.%d.%d\n", 
		fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

	printf( version );

	char timeStamp[512];
	sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

	printf( timeStamp );
}*/

static int help() {
    cout<<"/******** HELP *******/\n";
    cout << "\nThis program uses pointgrey cameras to capture and save stereo images.\n This program can generates .png, ppm, bmp images.\n";
    cout<<"\nKeyboard Shortcuts for usage:\n";
    cout<<"1. Default Mode: Capture images with 100 fps and save them in png format\n";
    cout<<"2. '-m': Starts capturing stereo images for calibration use with 2 fps\n";
    cout<<"3. '-f png': choose the format of captured images ,like png,jpg,ppm,bmp,etc";
    cout<<"\nType ./cap --help for more details.\n";
    cout<<"\n/******* HELP ENDS *********/\n\n";
    return 0;
}

void PrintCameraInfo_l( FlyCapture2::CameraInfo* pCamInfo )
{
	printf(
		"\n*** LEFT CAMERA INFORMATION ***\n"
		"Serial number - %u\n"
		"Camera model - %s\n"
		"Sensor - %s\n"
		"Resolution - %s\n"
		"Firmware version - %s\n"
		"Firmware build time - %s\n\n",
		pCamInfo->serialNumber,
		pCamInfo->modelName,
		pCamInfo->sensorInfo,
		pCamInfo->sensorResolution,
		pCamInfo->firmwareVersion,
		pCamInfo->firmwareBuildTime );
}

void PrintCameraInfo_r( FlyCapture2::CameraInfo* pCamInfo )
{
	printf(
		"\n*** RIGHT CAMERA INFORMATION ***\n"
		"Serial number - %u\n"
		"Camera model - %s\n"
		"Sensor - %s\n"
		"Resolution - %s\n"
		"Firmware version - %s\n"
		"Firmware build time - %s\n\n",
		pCamInfo->serialNumber,
		pCamInfo->modelName,
		pCamInfo->sensorInfo,
		pCamInfo->sensorResolution,
		pCamInfo->firmwareVersion,
		pCamInfo->firmwareBuildTime );
}

void PrintProperty_l( FlyCapture2::Property* pProp)
{
	printf(
		"\n*** LEFT PROPERTY INFORMATION ***\n"
		"type - %u\n"
		"abs control - %d\n"
		"one push - %d\n"
		"onoff - %d\n"
		"auto manual mode - %d\n"
		"abs value - %f\n\n",
		pProp->type,
		pProp->absControl,
		pProp->onePush,
		pProp->onOff,
		pProp->autoManualMode,
		pProp->absValue);
}

void PrintProperty_r( FlyCapture2::Property* pProp)
{
	printf(
		"\n*** RIGHT PROPERTY INFORMATION ***\n"
		"type - %u\n"
		"abs control - %d\n"
		"one push - %d\n"
		"onoff - %d\n"
		"auto manual mode - %d\n"
		"abs value - %f\n\n",
		pProp->type,
		pProp->absControl,
		pProp->onePush,
		pProp->onOff,
		pProp->autoManualMode,
		pProp->absValue);
}

void PrintError( FlyCapture2::Error error )
{
	error.PrintErrorTrace();
}

bool PollForTriggerReady( FlyCapture2::Camera* pCam )	//This is just for software trigger
{
	const unsigned int k_softwareTrigger = 0x62C;
	FlyCapture2::Error error;
	unsigned int regVal = 0;

	do 
	{
		error = pCam->ReadRegister( k_softwareTrigger, &regVal );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return false;
		}

	} while ( (regVal >> 31) != 0 );

	return true;
}

bool CreateDirectory(char *datechar, char *timechar)
{
	char* datestr = datechar;
	char* timestr = timechar;
	char folderstring[14];
	bool isDir;

	folderstring[0]=datestr[0];
	folderstring[1]=datestr[1];
	folderstring[2]=datestr[2];
	folderstring[3]=datestr[5];
	folderstring[4]=datestr[6];
	folderstring[5]=datestr[9];
	folderstring[6]=datestr[10];
	folderstring[7]=datestr[11];
	folderstring[8]=datestr[12];
	folderstring[9]=timestr[0];
	folderstring[10]=timestr[1];
	folderstring[11]=':';
	folderstring[12]=timestr[3];
	folderstring[13]=timestr[4];	

	printf("folderstring is %s \n",folderstring);
	char *home = getenv("HOME");
	char pathex[30];
	sprintf(pathex,"%s/experiments",home);
	if(NULL == opendir(pathex))
    mkdir(pathex,0777);
	sprintf(finalarray,"%s/%s",pathex,folderstring);
	mkdir(finalarray,0777);
	//makeing files
	//actually here we should check whether the directory exists using stat()
	struct stat statbuf;
	if (stat(finalarray, &statbuf) != -1) {
	   if (S_ISDIR(statbuf.st_mode)) {
	   		sprintf(txtfile,"%s/timing_file.txt",finalarray);
	      	return true;
	   }
	} else {
				return false;
	}
}