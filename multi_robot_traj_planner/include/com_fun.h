/*
//Title: Common funcation (Delta-NTU Lab)
//Author: Chen, Chun-Lin
//Data: 2015/06/17
//Update: 2016/03/31
*/

#ifndef COMM_H
#define COMM_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
using namespace std;
#define PI                      3.1415926
#define DTR                     PI/180.0        //Degree to Rad
#define RTD                     180.0/PI        //Rad to Degree

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>


//Euler angle to Quaternion
geometry_msgs::Quaternion Euler_to_Quat(double roll, double pitch, double yaw)
{
    tf::Quaternion tf_quat;
    geometry_msgs::Quaternion msg_quat;
    tf_quat=tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tf_quat, msg_quat);
    return msg_quat;
}

//Quaternion to one Euler angle
double Quat_to_Euler(geometry_msgs::Quaternion msg_quat, int sel)
{
    tf::Quaternion tf_quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg_quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    if (sel==0)
        return roll;
    else if (sel==1)
        return pitch;
    else if (sel==2)
        return yaw;
}

//Quaternion to Yaw
double Quat_to_Yaw(geometry_msgs::Quaternion msg_quat)
{
    double yaw;
    //TF Function Converte to Yaw Angle (Euler Angle)
    geometry_msgs::Pose msg_pose;
    msg_pose.orientation=msg_quat;
    tf::Pose tf_pose;
    tf::poseMsgToTF(msg_pose, tf_pose);
    yaw = tf::getYaw(tf_pose.getRotation());
    return yaw;
}

//Calculate Distance in XY plane
double DIS_XY(double pa_x, double pa_y, double pb_x, double pb_y)
{
    double distance;
    distance=pow(pow(pa_x-pb_x,2)+pow(pa_y-pb_y,2),0.5);
    return distance;
}

//Calculate Distance in XYZ plane
double DIS_XYZ(double pa_x, double pa_y, double pa_z, double pb_x, double pb_y, double pb_z)
{
    double distance;
    distance=pow(pow(pa_x-pb_x,2)+pow(pa_y-pb_y,2)+pow(pa_z-pb_z,2),0.5);
    return distance;
}
#endif // COMM_H
