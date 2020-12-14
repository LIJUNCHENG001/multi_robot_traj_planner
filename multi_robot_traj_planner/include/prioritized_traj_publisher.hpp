#pragma once

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <com_fun.h>

// MATPLOTLIB-CPP
#define _USE_MATH_DEFINES
#include <cmath>

// Submodules
#include <prioritized_traj_optimization.hpp>
#include <corridor.hpp>
#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>

class ResultPublisher {
public:
    ResultPublisher(ros::NodeHandle _nh,
                  std::shared_ptr<MPCPlanner> _MPCPlanner_obj,
                  std::shared_ptr<Corridor> _corridor_obj,
                  std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
                  SwarmPlanning::Mission _mission,
                  SwarmPlanning::Param _param)
            : nh(std::move(_nh)),
              MPCPlanner_obj(std::move(_MPCPlanner_obj)),
              corridor_obj(std::move(_corridor_obj)),
              initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        qn = mission.qn;
        outdim = 3;

        M=initTrajPlanner_obj.get()->T.size()-1;
        T.resize(N);

        for(int m = 0; m < N; m++){
            T[m] = DT*m;
        }

        traj_pubs.resize(qn);


        param.color[1][0]=0;
        param.color[1][1]=0;
        param.color[1][2]=1;


        for(int qi = 0; qi < qn; qi++){
            std::string mav_name = "/mav" + std::to_string(qi);
            traj_pubs[qi] = nh.advertise<nav_msgs::Path>("/desired_trajectory" + mav_name, 1);
        }
        
        initTraj_pub = nh.advertise<visualization_msgs::MarkerArray>("/initTraj", 1);
        obsBox_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_box", 1);
        samplepath_pub = nh.advertise<nav_msgs::Path>("/samplepath",1);
        colBox_pub = nh.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);
        colBox_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/collision_model2", 1);

        msgs_traj.resize(qn);

    }



    void update(double current_time){
        update_initTraj();
        update_obsBox(current_time);
        update_traj(current_time);
        update_colBox();
    }

    void publish(){
        for(int qi = 0; qi < qn; qi++){
            traj_pubs[qi].publish(msgs_traj[qi]);
        }
        initTraj_pub.publish(msgs_initTraj);
        obsBox_pub.publish(msgs_obsBox);
        colBox_pub.publish(msgs_colBox);
        colBox_pub2.publish(msgs_colBox2);
        samplepath_pub.publish(samplepath);
    }

    void plot(bool log) {

    }

private:
    ros::NodeHandle nh;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<MPCPlanner> MPCPlanner_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    int qn, M, outdim;
    double global_min_dist;
    tf::TransformBroadcaster br;
    std::vector<Eigen::MatrixXd> pva;
    std::vector<Eigen::MatrixXd> coef;
    std::vector<std::vector<double>> currentState;
    std::vector<double> T, t, max_dist, min_dist;
    std::vector<std::vector<std::vector<double>>> quad_state;

    // ROS publisher
    ros::Publisher traj_info_pub;
    std::vector<ros::Publisher> traj_coef_pubs;
    std::vector<ros::Publisher> traj_pubs;
    ros::Publisher initTraj_pub;
    ros::Publisher obsBox_pub;
    std::vector<ros::Publisher> relBox_pubs;
    ros::Publisher feasibleBox_pub;
    ros::Publisher colBox_pub;
    ros::Publisher colBox_pub2;
    ros::Publisher samplepath_pub;

    // ROS messages
    std::vector<nav_msgs::Path> msgs_traj;
    visualization_msgs::MarkerArray msgs_initTraj;
    visualization_msgs::MarkerArray msgs_obsBox;
    std::vector<visualization_msgs::MarkerArray> msgs_relBox;
    visualization_msgs::MarkerArray msgs_feasibleBox;
    visualization_msgs::MarkerArray msgs_colBox;
    visualization_msgs::MarkerArray msgs_colBox2;
    nav_msgs::Path samplepath;
    geometry_msgs::PoseStamped pp;

    Eigen::MatrixXd inter;


    void timeMatrix(double current_time, int& index, Eigen::MatrixXd& inter){
        double tseg = 0;
        double tcand;

        // find segment start time tseg
        for(int m = 0; m < N; m++){
            tcand = T[m];
            if(tcand < current_time){
                tseg = tcand;
                index = m;
            } else {
                break;
            }
        }
        tseg = current_time-tseg;
        tseg = tseg/DT;
        inter.resize(qn,3);
        for (int qi=0;qi<qn;qi++){
            for (int i=0;i<3;i++){
                inter(qi,i)=tseg*plan[qi][index+1][i]+(1-tseg)*plan[qi][index][i];
            }
        }
    }


    void update_traj(double current_time){
        if (current_time > T.back()) {
            return;
        }
        for(int qi = 0; qi < qn; qi++) {
            int index = 0;
            timeMatrix(current_time, index, inter);

            msgs_traj[qi].header.frame_id = "/world";
            msgs_traj[qi].header.stamp.sec = current_time;

            geometry_msgs::PoseStamped pos_des;
            pos_des.header.frame_id = "/world";
            pos_des.pose.position.x = inter(qi,0);
            pos_des.pose.position.y = inter(qi,1);
            pos_des.pose.position.z = 0;
            msgs_traj[qi].poses.emplace_back(pos_des);

        }
    }




    bool print_a=0;
    void update_initTraj(){
        visualization_msgs::MarkerArray mk_array;
        for (int qi = 0; qi < qn; qi++) {
            for (int m = 0; m < M; m++) {
                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.header.stamp = ros::Time::now();
                mk.ns = "mav" + std::to_string(qi);
                mk.type = visualization_msgs::Marker::CUBE;
                mk.action = visualization_msgs::Marker::ADD;

                mk.pose.orientation.x = 0.0;
                mk.pose.orientation.y = 0.0;
                mk.pose.orientation.z = 0.0;
                mk.pose.orientation.w = 1.0;

                mk.color.a = 1.0;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk.id = m;
                octomap::point3d p_init = initTrajPlanner_obj->initTraj[qi][m];
                octomap::point3d p_init2 = initTrajPlanner_obj->initTraj[qi][m+1];
                mk.pose.position.x = p_init.x();
                mk.pose.position.y = p_init.y();
                mk.pose.position.z = p_init.z();

                mk.scale.x = 0.1;
                mk.scale.y = 0.1;
                mk.scale.z = 0.1;

                mk_array.markers.emplace_back(mk);


                if ((!print_a) && (qi==1)){

                    samplepath.header.frame_id="world";
                    samplepath.header.stamp=ros::Time::now();
                    double dx=(p_init2.x()-p_init.x())/5;
                    double dy=(p_init2.y()-p_init.y())/5;

                    for (int j=0;j<5;j++)
                    {
                        pp.pose.position.x=p_init.x()+dx*j;
                        pp.pose.position.y=p_init.y()+dy*j;
                        samplepath.poses.emplace_back(pp);
                    }
                }
            }
        }
        msgs_initTraj = mk_array;
        print_a=1;
    }

    void update_obsBox(double current_time){
        visualization_msgs::MarkerArray mk_array;
        for (int qi = 0; qi < qn; qi++){
            // find current obsBox number
            int box_curr = 0;
            while(box_curr < corridor_obj->SFC[qi].size() &&
                    corridor_obj->SFC[qi][box_curr].second < current_time){
                box_curr++;
            }
            if(box_curr >= corridor_obj->SFC[qi].size()){
                box_curr = corridor_obj->SFC[qi].size() - 1;
            }

            visualization_msgs::Marker mk;
            mk.header.frame_id = "world";
            mk.ns = "mav" + std::to_string(qi);
            mk.type = visualization_msgs::Marker::CUBE;
            mk.action = visualization_msgs::Marker::ADD;

            mk.pose.orientation.x = 0.0;
            mk.pose.orientation.y = 0.0;
            mk.pose.orientation.z = 0.0;
            mk.pose.orientation.w = 1.0;

            for (int bi = 0; bi < corridor_obj->SFC[qi].size(); bi++){
                mk.id = bi;
//                mk.header.stamp = ros::Time(obstacle_boxes[qi][bi].second);
                std::vector<double> obstacle_box = corridor_obj->SFC[qi][bi].first;

                {
                    double margin = mission.quad_size[qi];
                    obstacle_box[0] -= margin;
                    obstacle_box[1] -= margin;
                    obstacle_box[2] -= margin;
                    obstacle_box[3] += margin;
                    obstacle_box[4] += margin;
                    obstacle_box[5] += margin;
                }

                mk.pose.position.x = (obstacle_box[0]+obstacle_box[3])/2.0;
                mk.pose.position.y = (obstacle_box[1]+obstacle_box[4])/2.0;
                mk.pose.position.z = (obstacle_box[2]+obstacle_box[5])/2.0;

                mk.scale.x = obstacle_box[3]-obstacle_box[0];
                mk.scale.y = obstacle_box[4]-obstacle_box[1];
                mk.scale.z = obstacle_box[5]-obstacle_box[2];

                mk.color.a = 0.2;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk_array.markers.emplace_back(mk);
            }
        }
        msgs_obsBox = mk_array;
    }

    // obstacle-collision model
    void update_colBox(){
        visualization_msgs::MarkerArray mk_array, mk_array2;
        for (int qi = 0; qi < qn; qi++) {
            

            for (int i=0;i<2;i++){
                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.ns = "colBox";
                if (i==0)
                {
                    mk.type = visualization_msgs::Marker::MESH_RESOURCE;
                    mk.mesh_resource = std::string("package://multi_robot_traj_planner/chassis.dae");

                    mk.scale.x=0.7;
                    mk.scale.y=0.7;
                    mk.scale.z=0.7;
                }
                else{
                    mk.type = visualization_msgs::Marker::ARROW;
                    mk.scale.x = 1.6 * mission.quad_size[qi];
                    mk.scale.y = 0.9 * mission.quad_size[qi];
                    mk.scale.z = 0.9 * mission.quad_size[qi];
                }
                mk.action = visualization_msgs::Marker::ADD;

                mk.id = qi;
                mk.pose.position.x = inter(qi,0);
                mk.pose.position.y = inter(qi,1);
                mk.pose.position.z = 0;

                mk.pose.orientation = Euler_to_Quat (0,0,inter(qi,2));

                mk.color.a = 0.7;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                if (i==0)
                    mk_array.markers.emplace_back(mk);
                else
                    mk_array2.markers.emplace_back(mk);

            }
            
        }
        msgs_colBox = mk_array;
        msgs_colBox2 = mk_array2;
    }
};