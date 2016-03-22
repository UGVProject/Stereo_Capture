#include "flycap.h"
// Opencv Library
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
//#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;

int main( int argc, char** argv )
{    
	time(&now);
	strftime(datestr, 20, "%b. %d, %Y", localtime(&currentdate));
	timenow = localtime(&now);
	strftime(timestr,20,"%H %M",timenow);
    std::cout<<"{help| |Prints this}\n";
    std::cout<<"{m calibrate mode: 0 normal mode (default mode); 2 calibration mode}\n";
    std::cout<<"{e image extention: p(ng) -- default, j(pg), b(mp), m(ppm)}\n";
    std::cout<<"{n image number to be captured (default n = 1000)\n"<<std::endl;

    for( int i =1; i < argc; i++)
    {
    	//if( argv[i] == 'm')
    	if(strcmp(argv[i],"m")==0)
    	{
            if( sscanf(argv[++i], "%d", &mode) != 1 || ((mode != 0) && (mode != 2) ))
            {
                std::cout << "invalid mode" << std::endl;
                return 0;
            }	
    	}
    	else if(strcmp(argv[i],"e")==0)
    	{
            if( sscanf(argv[++i], "%c", &extent) != 1 || ((extent != 'p') && (extent != 'j') && (extent != 'b')&& (extent != 'm')))
            {
                std::cout << "invalid format" << std::endl;
                return 0;
            }
            if(extent == 'j')
            {
            	extent3 = (char *)"jpg";
            }	
            else if(extent == 'b')
            {
            	extent3 = (char *)"bmp";
            }
            else if(extent == 'm')
            {
            	extent3 = (char *)"ppm";
            }
            else
            {
            	extent3 = (char *)"png";
            }
    	}
        else if(strcmp(argv[i],"n")==0)
    	{
            if( sscanf(argv[++i], "%d", &capNum) != 1 || (capNum < 0) )
            {
                std::cout << "invalid number" << std::endl;
                return 0;
            }
    	}
    }
	if(!CreateDirectory(datestr,timestr))
	{
		printf("Directory is not created!");
		return -1;
	}

	std::ofstream outputFile;
	outputFile.open(txtfile);

	FlyCapture2::BusManager busMgr;
	FlyCapture2::Error error;
	FlyCapture2::PGRGuid guid[2];
	FlyCapture2::Camera cam[2];
	FlyCapture2::CameraInfo camInfo[2];
	FlyCapture2::Property camProperty[2];
	FlyCapture2::Image rawImage[2]; 
	FlyCapture2::Image bgrImage[2];
	FlyCapture2::TriggerMode triggerMode[2];
	// opencv variable
	//raw8 format is 1280 * 1024 *1byte per pixel
	cv::Mat m_res[2];
	cv::Mat m_show=cv::Mat (1024,1280,CV_8UC1);
	std::vector<cv::Mat> rawleft;
	std::vector<cv::Mat> rawright;

	//limit the number of files to capNum
	rawleft.resize(capNum);
	rawright.resize(capNum);

	
	// opencv variables
	// Get the number of cameras
	error = busMgr.GetNumOfCameras( &numCameras );
	if (error != FlyCapture2::PGRERROR_OK
		&& numCameras >= 2
		)
	{
		PrintError( error );
		return -1;
	}
	//PrintBuildInfo();
	std::cout << "Number of cameras detected: " << numCameras << "\n" << std::endl; 
	
	// Get the camera information
	error = busMgr.GetCameraFromSerialNumber(15231263, &guid[0]);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	error = busMgr.GetCameraFromSerialNumber(15231302, &guid[1]);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	// Connect to a camera
	error = cam[0].Connect(&guid[0]);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	error = cam[1].Connect(&guid[1]);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	if(mode == 0){
		// Get the camera information	
		error = cam[0].GetCameraInfo(&camInfo[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		camProperty[0].type = FlyCapture2::SHUTTER;
		camProperty[0].absControl = true;
		camProperty[0].onePush = false;
		camProperty[0].onOff = true;
		camProperty[0].autoManualMode = false;
		camProperty[0].absValue = shuttlespeed;
		error = cam[0].SetProperty(&camProperty[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = cam[0].GetProperty(&camProperty[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		PrintProperty_l(&camProperty[0]);
	   
		error = cam[1].GetCameraInfo(&camInfo[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		camProperty[1].type = FlyCapture2::SHUTTER;
		camProperty[1].absControl = true;
		camProperty[1].onePush = false;
		camProperty[1].onOff = true;
		camProperty[1].autoManualMode = false;
		camProperty[1].absValue = shuttlespeed;
		
		error = cam[1].SetProperty(&camProperty[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = cam[1].GetProperty(&camProperty[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		PrintProperty_r(&camProperty[1]);

		error = cam[0].GetTriggerMode( &triggerMode[0] );

		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		
		// Set left camera to trigger mode 0
		triggerMode[0].onOff = true;
		triggerMode[0].mode = 0; //0
		triggerMode[0].parameter = 0;
		triggerMode[0].source = 0;
		triggerMode[0].polarity = 0;		//falling edge
		
		error = cam[0].SetTriggerMode( &triggerMode[0] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		error = cam[1].GetTriggerMode( &triggerMode[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		// Set right camera to trigger mode 0
		triggerMode[1].onOff = true;
		triggerMode[1].mode = 0;
		triggerMode[1].parameter = 0;
		triggerMode[1].source =0;	// 7 means software tirgger. 0 is camera external trigger
		triggerMode[1].polarity = 0;	//falling edge

		error = cam[1].SetTriggerMode( &triggerMode[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
			// Display Camera Settings
		PrintCameraInfo_l(&camInfo[0]);
		PrintCameraInfo_r(&camInfo[1]);
		std::cout << "Now Detecting Trigger Signals~~~~~~~~~ \n" << std::endl; 
	}
	else if(mode == 2)
	{
		error = cam[0].GetCameraInfo(&camInfo[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		camProperty[0].type = FlyCapture2::SHUTTER;
		camProperty[0].absControl = true;
		camProperty[0].onePush = false;
		camProperty[0].onOff = true;
		camProperty[0].autoManualMode = false;
		camProperty[0].absValue = 50;
		error = cam[0].SetProperty(&camProperty[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = cam[0].GetProperty(&camProperty[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		PrintProperty_l(&camProperty[0]);
	   
		error = cam[1].GetCameraInfo(&camInfo[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		camProperty[1].type = FlyCapture2::SHUTTER;
		camProperty[1].absControl = true;
		camProperty[1].onePush = false;
		camProperty[1].onOff = true;
		camProperty[1].autoManualMode = false;
		camProperty[1].absValue = 50;

		error = cam[1].SetProperty(&camProperty[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = cam[1].GetProperty(&camProperty[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		PrintProperty_r(&camProperty[1]);

		error = cam[0].GetTriggerMode( &triggerMode[0] );

		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		
		// Set left camera to trigger mode 0
		triggerMode[0].onOff = false;
		
		error = cam[0].SetTriggerMode( &triggerMode[0] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		error = cam[1].GetTriggerMode( &triggerMode[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		// Set right camera to trigger mode 0
		triggerMode[1].onOff = false;

		error = cam[1].SetTriggerMode( &triggerMode[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
			// Display Camera Settings
		PrintCameraInfo_l(&camInfo[0]);
		PrintCameraInfo_r(&camInfo[1]);
		std::cout << "Now We are Calibrating ... \n" << std::endl;
	}

	// Start capturing images
	error = cam[0].StartCapture();
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}

	error = cam[1].StartCapture();
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}
	cv::namedWindow("StereoCapture");   

	for(1;;)
	{       
		double _dwStart = GetTickCount();
		numImages ++;

		// Retrieve images		
		error = cam[0].RetrieveBuffer( &rawImage[0] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			continue;
		}
		error = cam[1].RetrieveBuffer( &rawImage[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			continue;
		}
		int A=std::abs(int(rawImage[0].GetTimeStamp().microSeconds-rawImage[1].GetTimeStamp().microSeconds));
		int B=std::abs(int((1e6-rawImage[0].GetTimeStamp().microSeconds)-rawImage[1].GetTimeStamp().microSeconds));
        
        //std::cout << "The timestamp for now is " << rawImage[0].GetTimeStamp().microSeconds << " haha \n" << std::endl;
		
	    if(min(A,B)>30)  	
		{
			numberofoutsync++;
		}

		Mat outofsyncimage(120, 500, CV_8UC3, Scalar(0,0,0));
		char displaymessage[64];
	    sprintf(displaymessage,"Out of sync for %d frames", numberofoutsync);
		cv::putText(outofsyncimage,displaymessage,cv::Point(20,80),FONT_HERSHEY_PLAIN,2,cv::Scalar(255,255,255),5);
		imshow("outbox",outofsyncimage);
		moveWindow("outbox",40,500);

		time_t rawtime;
        struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		
		struct tm * ptm;
		time ( &rawtime );
		ptm = gmtime ( &rawtime );
		
		long ms;
		struct timespec spec;
		clock_gettime(CLOCK_REALTIME, &spec);
		ms = round(spec.tv_nsec / 1.0e6);

		outputFile <<currentindex<<" "<<ptm->tm_year+1900<<" "<<ptm->tm_mon+1<<" "<<ptm->tm_mday<<" "<< (ptm->tm_hour+UTC)%24<<" "<< ptm->tm_min <<" "<< ptm->tm_sec<<" " << ms <<" "<< rawImage[1].GetTimeStamp().seconds <<" "<<  rawImage[1].GetTimeStamp().microSeconds<<" "<< rawImage[0].GetTimeStamp().seconds <<" "<<  rawImage[0].GetTimeStamp().microSeconds<<" " <<std::abs(int(rawImage[0].GetTimeStamp().microSeconds-rawImage[1].GetTimeStamp().microSeconds))<< std::endl;
		error = rawImage[0].Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &bgrImage[0] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			continue;
		}

		error = rawImage[1].Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &bgrImage[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			continue;
		}
				// convert to OpenCV Mat
		cv::Mat m_raw[2];
		unsigned int rowBytes = (double)bgrImage[0].GetReceivedDataSize() / (double)bgrImage[0].GetRows();       
		m_raw[0] = cv::Mat( bgrImage[0].GetRows(), bgrImage[0].GetCols(), CV_8UC1, 
			bgrImage[0].GetData(), rowBytes );
		m_raw[1] = cv::Mat( bgrImage[1].GetRows(), bgrImage[1].GetCols(), CV_8UC1, 
			bgrImage[1].GetData(), rowBytes );
		
		if( m_raw[0].cols > 640 ) 
		{
			cv::resize( m_raw[0], m_res[0], cv::Size( 640, 640*m_raw[0].rows/m_raw[0].cols ) );
			cv::resize( m_raw[1], m_res[1], cv::Size( 640, 640*m_raw[1].rows/m_raw[1].cols ) );
			
			
			m_show = cv::Mat( m_res[0].rows, m_res[0].cols*2, CV_8UC1);

			m_res[0].copyTo(m_show( cv::Rect( 0, 0, m_res[0].cols, m_res[0].rows ) ) );
			m_res[1].copyTo(m_show( cv::Rect( m_res[0].cols, 0, m_res[1].cols, m_res[1].rows ) ) );
		
		}

		cv::imshow("StereoCapture", m_show );
		input=cv::waitKey(1);

		m_raw[0].copyTo(rawleft[currentindex]);
		m_raw[1].copyTo(rawright[currentindex]);
		iterTime = (GetTickCount() - _dwStart);
		Sumiter += iterTime;

		if(mode == 0)
		{
			currentindex ++;
			if(1 == (numImages%20))
			{
				//std::cout<<std::abs(int(rawImage[0].GetTimeStamp().microSeconds-rawImage[1].GetTimeStamp().microSeconds))<<std::endl;			
				std::cout << "Grabbed image" << numImages << " @  " << (double)(1/iterTime*1000)  << "  fps\n" << std::endl;
			}
		}
		else if(mode == 2)
		{
			if(numImages%30 == 1)
			{
				currentindex ++;
				std::cout << "Grabbed image" << currentindex << " @  " << (double)(1/iterTime*100/3)  << "  fps\n" << std::endl;
			}
		}

		if((currentindex >= capNum) ||(input == 27))  //when the caputured number of images has come to the threshold or you press 'esc', the capture process ends.
		{
			outputFile.close();		
			Sumiter = Sumiter / currentindex;
			std::cout << "The average speed rate of the cameras is  " << (double)(1/Sumiter*1000) << "  fps\n" << std::endl;

			std::cout << "Stop Capturing" << std::endl;
			// Stop capturing images
			error = cam[0].StopCapture();
			if ( error != FlyCapture2::PGRERROR_OK )
			{
				PrintError( error );
				return -1;
			}

			error = cam[1].StopCapture();
			if ( error != FlyCapture2::PGRERROR_OK )
			{
				PrintError( error );
				return -1;
			}
			cv::destroyWindow("StereoCapture");

			waitKey(1);
			std::cout << "Set Trigger Mode\n" << std::endl;
			// Trigger
			triggerMode[0].onOff = false;    
			error = cam[0].SetTriggerMode( &triggerMode[0] );
			if (error != FlyCapture2::PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			triggerMode[1].onOff = false; 
			error = cam[1].SetTriggerMode( &triggerMode[1] );
			if (error != FlyCapture2::PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			std::cout << "Saving Pictures..." << std::endl;
			for(int i=0; i<currentindex;i++)
			{
				
				sprintf(filename0,"%s/L%05d.%s",finalarray,i,extent3);
				sprintf(filename1,"%s/R%05d.%s",finalarray,i,extent3);
				cv::imwrite(filename0,rawleft[i]);
				cv::imwrite(filename1,rawright[i]);
			}
			//sleep(1000);		
			std::cout << "Disconnect \n" << std::endl;
			// Disconnect the camera
			error = cam[0].Disconnect();
			if (error != FlyCapture2::PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			error = cam[1].Disconnect();
			if (error != FlyCapture2::PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			std::cout << "Done! Now exiting..." << std::endl;

			break;
		}
	}
	return 0;
}
