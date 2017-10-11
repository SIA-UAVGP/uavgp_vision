/*
 * @file	: CameraCoordinate2NegCoordinate.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"

static double distance_x = 0.695;
static double distance_y = 0;
static double distance_z = 0.15;

// zero is the lowest level, and 1 is the highest, the larger the lower level
static int accuracy_matrix[5][16] = 
    {1,1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,
    1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
    2,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
    4,4,4,5,6,7,8,9,10,11,12,13,14,15,16,17,
    6,6,6,6,7,8,9,10,11,12,13,14,15,16,17,18};

int GetTargetAccuracyLevel(Point3f cameraPos3D)
{
    float effective_distance[2] = {0.75f,4.75f};
    float dist_divided_num = 16.0f;
    float effective_angle = 36.0f;
    float angle_divided_num = 5.0f;
    int level = 0;
    
    float angle = atanf(sqrtf(cameraPos3D.x*cameraPos3D.x + cameraPos3D.y*cameraPos3D.y)/cameraPos3D.z)
                  *180.0f/3.14159f;
    
    float x = angle / effective_angle * angle_divided_num;
    float y = (cameraPos3D.z - effective_distance[0])/ (effective_distance[1] - effective_distance[0]) * dist_divided_num;

    if (x > 0.0f && x < angle_divided_num && y > 0.0f && y < dist_divided_num){
        level = accuracy_matrix[int(x)][int(y)];
    } else {
        level = 0; 
    }
    // printf("x = %4.2f, y = %4.2f, level = %d \n", x, y, level);
    return level;
}

void CameraCoordinate2NegCoordinate( vector<VisionResult>& vision_results, const Attitude3D& attitude3d)
{
    double port_attitude_roll  = (double)attitude3d.roll;
    double port_attitude_pitch = (double)attitude3d.pitch;
    double port_attitude_yaw   = (double)attitude3d.yaw;

    for(int i=0;i<(int)vision_results.size();++i)
    {
        double cam_x = (double)vision_results[i].cameraPos3D.x;
        double cam_y = (double)vision_results[i].cameraPos3D.y;
        double cam_z = (double)vision_results[i].cameraPos3D.z;

        double uav_x = 0;
        double uav_y = 0;
        double uav_z = 0;

        uav_x=cam_z + distance_x;
        uav_y=cam_x + distance_y;
        uav_z=cam_y + distance_z;

        double sinx,siny,sinz,cosx,cosy,cosz;
        sinx=sin(port_attitude_roll); siny=sin(port_attitude_pitch); sinz=sin(port_attitude_yaw);
        cosx=cos(port_attitude_roll); cosy=cos(port_attitude_pitch); cosz=cos(port_attitude_yaw);
        double uav_neg_x,uav_neg_y,uav_neg_z;

        uav_neg_x = uav_x*cosy*cosz + uav_y*(sinx*siny*cosz - cosx*sinz) + uav_z*(cosx*siny*cosz + sinx*sinz);
        uav_neg_y = uav_x*cosy*sinz + uav_y*(sinx*siny*sinz + cosx*cosz) + uav_z*(cosx*siny*sinz - sinx*cosz);
        uav_neg_z = -uav_x*siny + uav_y*sinx*cosy + uav_z*cosx*cosy;

        vision_results[i].negPos3D.x = uav_neg_x;
        vision_results[i].negPos3D.y = uav_neg_y;
        vision_results[i].negPos3D.z = uav_neg_z;
        vision_results[i].accuracy_level = GetTargetAccuracyLevel(vision_results[i].cameraPos3D);
        ROS_INFO("Num: %d; Location: %4.2f, %4.2f, %4.2f; Accuracy: %d",
                vision_results[i].digitNo, cam_x, cam_y, cam_z, vision_results[i].accuracy_level);
    }
    return;
}
