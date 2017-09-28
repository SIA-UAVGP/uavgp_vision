#include <bitset>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sensor_msgs/LaserScan.h>
#include "DetectRectToGetImageLightness.h"
#include "../include/FlyCapture2.h"
#include "project_path_config.h"
#include <std_msgs/Int32.h>

#define  DISPLAYSCREEN 0
#define  PRINTBOARD 1
using namespace FlyCapture2;
using namespace std;
using namespace cv;
//相机帧率，单位：HZ
#define CameraFrameRate  19.0
static char baseDir[100] = TXT_FILE_PATH;

static int targetType = PRINTBOARD; //PRINTBOARD; DISPLAYSCREEN
float min_exposure_time = 0.8;
float max_exposure_time = 1.2;

float track_bar_shutter_time = 0;
int shutter_slider = 0;
int shutter_slider_max = 100;
//是否为室外环境
bool isOutdoor = true;
//设定的画面亮度值
double brightnessSet = 170;
//室内：maxShutterVal=30， 室外：maxShutterVal=10
float maxShutterVal;
//使能一键半自动自动曝光
bool adjustShutterTimeFlag = true;
//使能相机自动曝光
bool cameraAutoShutterFlag = true;
//曝光时间，单位：ms
float cameraExposureTime = 5.00;
//设置相机曝光增益
bool autoAddExposureFlag = true;
//曝光增益初始值
float addExposureVal = 0.01;
//自动曝光设置完成
bool adjustShutterTimeDone = false;

extern char pressedKey[20];
//计算曝光时间
double CalculateExposureTime( Mat& currentImg, const double currentExposureTime,  double brightnessSet, const double rectAreaBrightness );
void OpenGetKeyThread();
double HSVCenterROI( Mat& srct );
void on_trackbar( int, void* );

void SaveExposureTimeValueToTxt(char* baseDir, bool isOutdoor, float brightness, float exposureTimeValue)
{
    FILE* fp;
    char txtName[1000];
    sprintf(txtName, "%s/ExposureTimeValueInfo.txt", baseDir);
    fp = fopen(txtName,"a+");
    if (NULL == fp)
    {
        printf("Open txt failed ...\n");
        exit(-2);
    }

    char outdoor_name[20];
    if (true == isOutdoor)
        sprintf(outdoor_name,"outdoor");
    else
        sprintf(outdoor_name,"indoor");
    time_t  now;
    struct tm* timenow;
    time(&now);
    timenow = localtime(&now);
    char time_name[1000];
    sprintf(time_name,"%04d%02d%02d_%02d%02d_%02d",timenow->tm_year+1900,timenow->tm_mon+1,timenow->tm_mday,timenow->tm_hour,timenow->tm_min,timenow->tm_sec);    

    fprintf(fp,"%s %s %f %f\n", time_name, outdoor_name, brightness, exposureTimeValue);
    fclose(fp);
    return;
}


void PrintBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion( &fc2Version );

	ostringstream version;
	version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
	cout << version.str() << endl;

	ostringstream timeStamp;
	timeStamp <<"Application build date: " << __DATE__ << " " << __TIME__;
	cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
	cout << endl;
	cout << "*** CAMERA INFORMATION ***" << endl;
	cout << "Serial number -" << pCamInfo->serialNumber << endl;
	cout << "Camera model - " << pCamInfo->modelName << endl;
	cout << "Camera vendor - " << pCamInfo->vendorName << endl;
	cout << "Sensor - " << pCamInfo->sensorInfo << endl;
	cout << "Resolution - " << pCamInfo->sensorResolution << endl;
	cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
	cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;

}

void PrintError( FlyCapture2::Error error )
{
	error.PrintErrorTrace();
}



void PrintFormat7Capabilities( Format7Info fmt7Info )
{
    cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
    cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
    cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;

}


//format7，设置最大分辨率
int SetFormat7(Camera& cam)
{
    return 1;
}

//设置相机Gain增益
int SetGainMode( Camera& cam )
{
        FlyCapture2::Error error;
        Property prop;

        prop.type = GAIN;
        error = cam.GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
           PrintError( error );
           return -1;
        }
        prop.type = GAIN;
        prop.absControl = true;
        prop.onOff = true;
        prop.autoManualMode = false;
        prop.onePush = false;
        if ( false == isOutdoor )
            prop.absValue = 20.0;
        else
            prop.absValue = 0.0;

        error = cam.SetProperty( &prop);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
        return 1;
}

//设置相机输出频率
int SetCameraFrameRate( Camera& cam )
{
        FlyCapture2::Error error;

        Property prop;
        prop.type = FRAME_RATE;
        error = cam.GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
           PrintError( error );
           return -1;
        }
        prop.type = FRAME_RATE;
        prop.absControl = true;
        prop.onOff = true;
        prop.autoManualMode = false;
        prop.onePush = false;                           // onePush统统false
        float frameVal = CameraFrameRate;
        prop.absValue = frameVal;
        error = cam.SetProperty( &prop);
        if (error != PGRERROR_OK)
        {
          PrintError( error );
          return -1;
        }
        return 1;
}

//设置相机曝光时间
int SetCameraExposureTime( Camera& cam, bool cameraAutoShutterFlag, float shutterVal  )
{
        FlyCapture2::Error error;
        Property prop;

        //设置快门时间
        prop.type = SHUTTER;
        error = cam.GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
           PrintError( error );
           return -1;
        }
        prop.type = SHUTTER;
        prop.absControl = true;
        prop.onOff = true;
        prop.autoManualMode = cameraAutoShutterFlag;
        prop.onePush = false;
        if(shutterVal < 100)
             prop.absValue = shutterVal;
        error = cam.SetProperty( &prop);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
        return 1;
}

//设置相机曝光增益
int SetExposureMode( Camera& cam, bool autoAddExposureFlag, float addExposureVal )
{
        FlyCapture2::Error error;
        Property prop;

        prop.type = AUTO_EXPOSURE;
        error = cam.GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
           PrintError( error );
           return -1;
        }
        prop.type = AUTO_EXPOSURE;
        prop.absControl = true;
        prop.onOff = true;
        prop.autoManualMode = autoAddExposureFlag;
        prop.onePush = false;
        prop.absValue = addExposureVal;
        if(true == isOutdoor)
        {
            prop.autoManualMode = false;
            prop.absValue = 0.01;
        }
        error = cam.SetProperty( &prop);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
        return 1;
}

//////半自动曝光
void AdjustShutterTime(Camera& cam, float& shutterVal, Mat& currentImg, const double rectAreaBrightness )
{
    if (true == autoAddExposureFlag)
    {
        //关闭自动曝光增益
        autoAddExposureFlag = false;
        addExposureVal = 0.01;
        SetExposureMode( cam, autoAddExposureFlag, addExposureVal );
    }

    Property prop;
    prop.type = SHUTTER;
    cam.GetProperty( &prop );

    float oldShutterVal = prop.absValue;
    shutterVal = CalculateExposureTime( currentImg,  oldShutterVal,  brightnessSet, rectAreaBrightness );
    if ( fabs(shutterVal - oldShutterVal) < 0.00001)
    {
        adjustShutterTimeDone = true;
        return;
    }

    if ( (shutterVal > 20 || shutterVal < 0.02) && false == autoAddExposureFlag )
    {
        //开启自动曝光增益
        autoAddExposureFlag = true;
        addExposureVal = 0.01;
        SetExposureMode( cam, autoAddExposureFlag, addExposureVal );
    }

    cameraAutoShutterFlag = false;
    if (shutterVal > maxShutterVal)
            shutterVal = maxShutterVal;
    SetCameraExposureTime( cam, cameraAutoShutterFlag, shutterVal );

    printf("shutter = %f\n", shutterVal);
    return;
}


/* subscribe camera_switch_data from state_machine offb_simulation_test node. */
static std_msgs::Int32 camera_switch;
static void camera_switch_cb(const std_msgs::Int32::ConstPtr& msg)
{
    camera_switch = *msg;
    if(1 == camera_switch.data)
        targetType = DISPLAYSCREEN;
    if(2 == camera_switch.data)
        targetType = PRINTBOARD;
    ROS_INFO("get camera_switch_data = %d",camera_switch.data);
}

////相机配置与图像采集
int RunSingleCamera( PGRGuid guid )
{
    if (false == isOutdoor)
        maxShutterVal = 30;
    else
        maxShutterVal = 10;

    ros::NodeHandle rawImgPubNode;
    image_transport::ImageTransport it(rawImgPubNode);
    image_transport::Publisher pub;
    pub = it.advertise("vision/camera_image",  1 );

    ros::Publisher camera_info_pub;
    camera_info_pub = rawImgPubNode.advertise<sensor_msgs::LaserScan>("vision/camera_info", 1);

    FlyCapture2::Error error;
    Camera cam;
	// Connect to a camera
	error = cam.Connect(&guid);
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	// Get the camera information
	CameraInfo camInfo;
	error = cam.GetCameraInfo(&camInfo);
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	PrintCameraInfo(&camInfo);

    //format7，设置最大分辨率
    const Mode k_fmt7Mode = MODE_0;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
    // Query for available Format 7 modes
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = cam.GetFormat7Info( &fmt7Info, &supported );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    PrintFormat7Capabilities( fmt7Info );
    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
    {
        // Pixel format not supported!
        cout << "Pixel format is not supported" << endl;
        return -1;
    }
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

    bool valid;
    Format7PacketInfo fmt7PacketInfo;
    // Validate the settings to make sure that they are valid
    error = cam.ValidateFormat7Settings(
            &fmt7ImageSettings,
            &valid,
            &fmt7PacketInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    if ( !valid )
    {
        // Settings are not valid
        cout << "Format7 settings are not valid" << endl;
        return -1;
    }
    error = cam.SetFormat7Configuration(
            &fmt7ImageSettings,
            fmt7PacketInfo.recommendedBytesPerPacket );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    //设置相机Gain增益
    SetGainMode( cam );
    //设置相机输出频率
    SetCameraFrameRate( cam );
    //设置相机曝光时间
    SetCameraExposureTime( cam, cameraAutoShutterFlag,cameraExposureTime );
    //设置相机曝光增益
    SetExposureMode( cam, autoAddExposureFlag, addExposureVal );

	// Get the camera configuration
	FC2Config config;
    error = cam.GetConfiguration( &config );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	// Set the number of driver buffers used to 10.
    config.numBuffers = 10;
	// Set the camera configuration
	error = cam.SetConfiguration( &config );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	// Start capturing images
    error = cam.StartCapture();
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
    }

	Image rawImage;
    Image convertedImage;
    int imageCnt=0;
    printf("Start capture image from camera repeadly...\n");
    if ( true == adjustShutterTimeFlag )
            OpenGetKeyThread();

    DetectRectToGetImageLightness rectAreaLightness;
    double brightness;

    namedWindow("CameraCapture", 1);
    char TrackbarName[50];
    sprintf( TrackbarName, "shutter: %d", shutter_slider_max );
    createTrackbar( TrackbarName, "CameraCapture", &shutter_slider, shutter_slider_max, on_trackbar );
    on_trackbar( shutter_slider, 0 );

    float shutter_time = 0;
    while( rawImgPubNode.ok() )
	{
		// Retrieve an image
		error = cam.RetrieveBuffer( &rawImage );
		if (error != PGRERROR_OK)
		{
            //PrintError( error );
			continue;
		}
        FlyCapture2::Error error;
        Property prop;
        prop.type = SHUTTER;
        error = cam.GetProperty( &prop );
        shutter_time = prop.absValue;

        if (DISPLAYSCREEN == targetType)
        {
            //printf("targetType = DISPLAYSCREEN;\n");

            if (track_bar_shutter_time-0 > 0.1)
            {
                cameraAutoShutterFlag = false;
                cameraExposureTime = track_bar_shutter_time;
                SetCameraExposureTime( cam, cameraAutoShutterFlag,cameraExposureTime );
            }
            else
            {
                if (0 == imageCnt%54)
                {
                    cameraAutoShutterFlag = true;
                    cameraExposureTime = min_exposure_time;
                    SetCameraExposureTime( cam, cameraAutoShutterFlag,cameraExposureTime );

                    Image rawImageTemp;
                    while(1)
                    {
                        // Retrieve an image
                        error = cam.RetrieveBuffer( &rawImageTemp );
                        if (error == PGRERROR_OK)
                        {
                            break;
                        }
                    }
                }

                prop.type = SHUTTER;
                error = cam.GetProperty( &prop );
                shutter_time = prop.absValue;

                if (shutter_time < min_exposure_time)
                {
                    cameraAutoShutterFlag = false;
                    cameraExposureTime = min_exposure_time;
                    SetCameraExposureTime( cam, cameraAutoShutterFlag,cameraExposureTime );
                }
                else if (shutter_time >= max_exposure_time)
                {
                    cameraAutoShutterFlag = false;
                    cameraExposureTime = max_exposure_time;
                    SetCameraExposureTime( cam, cameraAutoShutterFlag,cameraExposureTime );
                }
            }

        }

        if (PRINTBOARD == targetType)
        {
            //printf("targetType = PRINTBOARD;\n");
            cameraAutoShutterFlag = true;
            cameraExposureTime = 1.0;
            SetCameraExposureTime( cam, cameraAutoShutterFlag,cameraExposureTime );
        }

		// Convert the raw image
        error = rawImage.Convert( PIXEL_FORMAT_RGB8, &convertedImage );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
            printf("rawImage Convert error...\n");
            //return -1;
		}

        Mat img = Mat::zeros(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3);
        img.data = convertedImage.GetData();
        cvtColor(img,img,CV_RGB2BGR);
        //printf("img.cols=%d; img.rows=%d\n", img.cols,img.rows);

        if ( true == adjustShutterTimeFlag && false == adjustShutterTimeDone )
        {
            if ( -1 == pressedKey[0] )
            {
                //HSVCenterROI( img );

                //detect rect for auto set exposure
                brightness = rectAreaLightness.GetRectAreaLightness(img);
                continue;
            }
            else if ( 'e' != pressedKey[0] )
            {
                adjustShutterTimeFlag = false;
                continue;
            }
            else
            {
                //detect rect for auto set exposure
                double brightness = rectAreaLightness.GetRectAreaLightness(img);
                if (brightness < 1)
                    continue;

                AdjustShutterTime(cam, cameraExposureTime, img, brightness);
                continue;
            }
        }
        //save exposure time
        static bool txtHasWriten = false;
        if (false == txtHasWriten)
        {
            Property prop;
            prop.type = SHUTTER;
            cam.GetProperty( &prop );
            cameraExposureTime = prop.absValue;
            SaveExposureTimeValueToTxt(baseDir, isOutdoor, (float)brightness, cameraExposureTime);
            txtHasWriten = true;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg);

        sensor_msgs::LaserScan cam_info;
        cam_info.ranges.resize(1);
        cam_info.header.frame_id = "cam_info";
        cam_info.header.stamp    = ros::Time::now();
        cam_info.ranges[0] = shutter_time;
        camera_info_pub.publish(cam_info);
        printf("shutter time: %f\n", shutter_time);

        resize(img, img, Size(640,480));
        imshow("CameraCapture",img);
        int c = waitKey(1);
        if (113 == c)
            break;
        if (imageCnt < 10)
        {
            printf("GrabNo = %d\n",  imageCnt);
        }
        imageCnt++;
        //printf("GrabNo = %d\n",  imageCnt);
        ros::spinOnce();
	}

	// Stop capturing images
	error = cam.StopCapture();
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	// Disconnect the camera
	error = cam.Disconnect();
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	return 0;
}


////主函数
int main(int argc, char **argv)
{
    //ros初始化
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle camera_capture_switch_sub_node;
    camera_switch.data = DISPLAYSCREEN;
    ros::Subscriber camera_capture_switch_sub = camera_capture_switch_sub_node.subscribe("camera_switch", 3, camera_switch_cb);

    FlyCapture2::Error error;
	PrintBuildInfo();

	BusManager busMgr;
	unsigned int numCameras;
	error = busMgr.GetNumOfCameras(&numCameras);
	if (error != PGRERROR_OK)
	{
		PrintError( error );
        //return -1;
	}

    printf("Number of cameras detected: %d\n", numCameras);
    if ( 0 == numCameras)
    {
        exit(-3);
    }

	for (unsigned int i=0; i < numCameras; i++)
	{
		PGRGuid guid;
		error = busMgr.GetCameraFromIndex(i, &guid);
		if (error != PGRERROR_OK)
		{
			PrintError( error );
            //return -1;
		}

		RunSingleCamera( guid );
	}

    //return 0;
}



void on_trackbar( int, void* )
{
 track_bar_shutter_time = shutter_slider*0.2;
 return;
}
