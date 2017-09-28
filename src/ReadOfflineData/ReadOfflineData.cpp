/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */
#include <opencv2/opencv.hpp>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;

char baseDir[1000] = "/home/sia/vision_ws/src/uavgp_vision/output/Save/20161123_1356_25";
unsigned int imgNo = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_raw_data_save");

    ros::NodeHandle rawPubNode;
    image_transport::ImageTransport it(rawPubNode);
    image_transport::Publisher pub;
    pub = it.advertise("vision/camera_image",  1 );

    int rate = 5;
    ros::Rate loop_rate( rate );
    char video_name[1000];
    sprintf(video_name,"%s/raw-000000.avi", baseDir);

    VideoCapture video_capture(video_name);

    FILE* fp;
    char txtName[1000];
    sprintf(txtName, "%s/switch_mode.txt", baseDir);
    fp = fopen(txtName,"r");
    if (NULL == fp)
    {
          printf("Open txt failed ...\n");
          //exit(-2);
    }

    ros::Publisher switch_pub;

    while( rawPubNode.ok() )
    {
        Mat raw_image;
        video_capture >> raw_image;
        char raw_img_name[1000];
        sprintf(raw_img_name,"%s/image-%06d.jpg",baseDir,imgNo);
        //raw_image = imread(raw_img_name);
        if (!raw_image.data)
        {
            cout<<"open image failed!"<<endl;
            break;
        }
        if (imgNo < 10*60*4.5)   //10*60*7.0
        {
            imgNo++;
            continue;
        }

        Mat resied_img;
        resize(raw_image,resied_img,Size(1384*0.4,1032*0.4));
        imshow("raw",resied_img);
        waitKey(1);
        printf("imgNo=%d\n", imgNo);

        int num;
        std_msgs::Int32 camera_switch_data;
        //if (EOF == fscanf(fp, "%d%d", &num,&camera_switch_data.data))
        //    break;

        switch_pub.publish(camera_switch_data);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_image).toImageMsg();
        pub.publish(msg);

        char img_name[1000];
        static int save_image_num = 400;
        sprintf(img_name,"%s/image/image-%06d.jpg",baseDir,save_image_num);
        //imwrite(img_name, raw_image);
        save_image_num++;

        loop_rate.sleep();
        imgNo++;
    }

    return 0;
}
