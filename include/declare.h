/*
 * @file	: declare.h
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "DisplayScreenProcess.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <time.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <boost/thread.hpp>
//#include <math.h>
//#include <linux/videodev2.h>

#define  OPENCV_VIDEOCAPTURE    0
#define  MVMODE                 1
#define  POINTGRAYCAMERA        2
#define  OFFLINEDATA            3

using namespace std;
using namespace cv; 

class CapturedImage
{
public:
    Mat rawImg;
};

class RectMark
{
public:
	RectMark(void)
	{
		indexId = -1;
		validFlag = false;
		minSideLength = 0;	
		maxSideLength = 0;	
		area = 0;
        rectKind = -1;
        digitNo = -1;
    }
    int frameNo;
    int indexId;                    //Rect编号
	bool validFlag;					//该Rect是否有效
    //矩形类别，矩形检测到后的正常取值：0， 1， 2
    int rectKind;
    int digitNo;

	//contour信息
    float minSideLength;            //最小边长
    float maxSideLength;            //最大边长
    float area;                     //面积
	vector<Point2f> m_points;		//四个顶点位置
    Point3f position;

	//透视变换后的图像
	Mat perspectiveImg;
    //对透视变换后的二值化处理后图像
    Mat possibleRectBinaryImg;
	//待数字识别图像
	Mat possibleDigitBinaryImg;
};

class Attitude3D
{
public:
    Attitude3D()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
    float roll;
    float pitch;
    float yaw;
};

//最终结果的自定义数据类型
class VisionResult
{
public:
    VisionResult( void )
    {
        frameNo = -1;
        digitNo = -1;
    }
    int frameNo;
    int digitNo;
    int accuracy_level;
    Point2f imagePos2D;
    Point3f cameraPos3D;
    Point3d negPos3D;
};

//ultrasonic data
class GuidanceDistance
{
public:
    double vbus_1_distance;
    double vbus_1_reliability;
    double vbus_2_distance;
    double vbus_2_reliability;
    double vbus_3_distance;
    double vbus_3_reliability;
    double vbus_4_distance;
    double vbus_4_reliability;
    double vbus_5_distance;
    double vbus_5_reliability;
};

//read offline data
int ReadOfflineData(  );
//创建ros消息发布的各个线程
void CreateRosPublishThread(const char* dir);
//开启ros发布图像线程
void*  RosImagePublishThread(void*);
//创建文件夹
int CreateDir(char* saveDir);
//Save
int CreatSaveDir (char* dir , bool saveImgFlag);
//对透视变换后的图像做处理，判断出检测到的矩形的类别
void GetRectKinds( vector< vector<RectMark> >&  rectCategory );
//求四边形内侧夹角
double GetTwoSideAngle(Point2f p1,Point2f p2, Point2f p3);
//显示时间、帧率
void ShowTime(Mat& inputImg, int frameNo, float shrink);
//
double GetAllPixelAverageValue(Mat& img);
//视觉检测结果保存为txt
void SaveResultToTxt(char* baseDir,  float shrink, vector<VisionResult>& result);
//Guidance
void DepthTo3D( Mat& depth_img, Mat& xyz_img);
void ConvertToPseudoColor( Mat& mat_xyz, Mat& img_pseudo_color );
//
double GetROI_AverageVal( Mat src, Point point, int channel, int radius);
//根据累积误差检测矩形
int RectDetectByStatisticsError(Mat& lightness_img, Mat& input_img, vector< vector<VisionResult> >& lastValidResult, vector<VisionResult>& incompleteRectResult);
