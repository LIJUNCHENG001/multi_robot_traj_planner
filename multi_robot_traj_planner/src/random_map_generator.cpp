#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Eigen>
#include <math.h>
#include <random>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <mission.hpp>


using namespace std;

pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double>  rand_x;
uniform_real_distribution<double>  rand_y;
uniform_real_distribution<double>  rand_w;
uniform_real_distribution<double>  rand_h;

ros::Publisher all_map_pub;

vector<double> _state;

int obs_num;
double margin;
double x_min, y_min, z_min, x_max, y_max, z_max;
double r_min, r_max, h_min, h_max, resolution;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

void RandomMapGenerate(const SwarmPlanning::Mission& mission)
{
    double numel_e = 0.00001;
    pcl::PointXYZ pt_random;

    rand_x = uniform_real_distribution<double>(x_min, x_max);
    rand_y = uniform_real_distribution<double>(y_min, y_max);
    rand_w = uniform_real_distribution<double>(r_min, r_max);
    rand_h = uniform_real_distribution<double>(h_min, h_max);

    int obs_iter = 0;
    while(obs_iter < obs_num)
    {
        double x, y, w, h;
        x    = rand_x(eng);
        y    = rand_y(eng);
        w    = rand_w(eng);

        bool quadInObs = false;
        for (int qi = 0; qi < mission.qn; qi++) {
            if (sqrt(pow(x - mission.startState[qi][0], 2) + pow(y - mission.startState[qi][1], 2)) < mission.quad_size[qi] + w + margin ||
                sqrt(pow(x - mission.goalState[qi][0], 2) + pow(y - mission.goalState[qi][1], 2)) < mission.quad_size[qi] + w  + margin){
                quadInObs = true;
                break;
            }
        }
        if(quadInObs){
            continue;
        }
        x = floor(x/resolution) * resolution + resolution / 2.0;
        y = floor(y/resolution) * resolution + resolution / 2.0;

        int widNum = ceil(w/resolution);

        for(int r = -widNum/2.0; r < widNum/2.0; r ++ ) {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / resolution);
                for (int t = 0; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.5) * resolution + numel_e;
                    pt_random.y = y + (s + 0.5) * resolution + numel_e;
                    pt_random.z = (t + 0.5) * resolution + numel_e;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }

        obs_iter++;
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generate random map ");

    kdtreeLocalMap.setInputCloud( cloudMap.makeShared() );
}

void pubSensedPoints()
{
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    all_map_pub.publish(globalMap_pcd);
}


int main (int argc, char** argv) {
    ros::init (argc, argv, "random_map_generator");
    ros::NodeHandle n( "~" );
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("all_map", 1);

    n.param<double>("world/x_min", x_min, -5);
    n.param<double>("world/y_min", y_min, -5);
    n.param<double>("world/z_min", z_min, 0);
    n.param<double>("world/x_max", x_max, 5);
    n.param<double>("world/y_max", y_max, 5);
    n.param<double>("world/z_max", z_max, 2.5);
    n.param<double>("world/margin", margin, 1.5);

    n.param<int>("world/obs_num", obs_num,  6);
    n.param<double>("world/resolution",  resolution, 0.1);
    n.param<double>("world/r_min", r_min,   0.3);
    n.param<double>("world/r_max", r_max,   0.8);
    n.param<double>("world/h_min", h_min,   1.0);
    n.param<double>("world/h_max", h_max,   2.5);

    SwarmPlanning::Mission mission;
    if(!mission.setMission(n)){
        return -1;
    }

    // generate map msg
    RandomMapGenerate(mission);

    ros::Rate rate(10);
    int count = 0;
    while (ros::ok())
    {
        if(count < 100) {
            pubSensedPoints();
            count++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
