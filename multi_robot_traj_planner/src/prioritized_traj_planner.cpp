// ROS
#include <ros/ros.h>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Parameters
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodules
#include <ecbs_planner.hpp>
#include <corridor.hpp>
#include <prioritized_traj_optimization.hpp>
#include <prioritized_traj_publisher.hpp>

bool has_octomap = false;
bool has_path = false;
std::shared_ptr<octomap::OcTree> octree_obj;

void octomapCallback(const octomap_msgs::Octomap& octomap_msg)
{
    if(has_octomap)
        return;

    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    has_octomap = true;
}

int main(int argc, char* argv[]) {

    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Subscriber octomap_sub = nh.subscribe( "/octomap_full", 1, octomapCallback );

    
    // Mission
    SwarmPlanning::Mission mission;
    if(!mission.setMission(nh)){
        return -1;
    }

    int qn=mission.qn;
    way_point_pub.resize(qn);
    for(int qi = 0; qi < qn; qi++){
            std::string robot_name = "/robot" + std::to_string(qi+1);
            way_point_pub[qi] = nh.advertise<nav_msgs::Path>(robot_name+"/mpc_predict_all", 1);
        }

    // ROS Parameters
    SwarmPlanning::Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(mission.qn);

    // Submodules
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<MPCPlanner> MPCPlanner_obj;
    std::shared_ptr<ResultPublisher> resultPublisher_obj;

    // Main Loop
    ros::Rate rate(20);
    Timer timer_total;
    Timer timer_step;
    double start_time, current_time;

    while (ros::ok()) {
        if (has_octomap && !has_path) {

            // Build 3D Euclidean Distance Field
            {
                float maxDist = 1;
                octomap::point3d min_point3d(param.world_x_min, param.world_y_min, param.world_z_min);
                octomap::point3d max_point3d(param.world_x_max, param.world_y_max, param.world_z_max);
                distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
                distmap_obj.get()->update();
            }

            timer_total.reset();
            ROS_INFO("Multi-robot Trajectory Planning");

            // Step 1: Plan Initial Trajectory
            timer_step.reset();
            {
                initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, mission, param));
                if (!initTrajPlanner_obj.get()->update(param.log)) {
                    return -1;
                }
            }
            timer_step.stop();
            ROS_INFO_STREAM("ECBS Planner runtime: " << timer_step.elapsedSeconds());

            
            // Step 2: Generate Safe Corridor
            timer_step.reset();
            {
                corridor_obj.reset(new Corridor(initTrajPlanner_obj, distmap_obj, mission, param));
                if (!corridor_obj.get()->update(param.log)) {
                    return -1;
                }
            }
            timer_step.stop();
            ROS_INFO_STREAM("Safe Corrifor runtime: " << timer_step.elapsedSeconds());
            
            
            // Step 3: Formulate NLP problem and solving it to generate trajectory for the robot team
            timer_step.reset();
            {
                MPCPlanner_obj.reset(new MPCPlanner(corridor_obj, initTrajPlanner_obj, mission, param));
                if (!MPCPlanner_obj.get()->update(param.log)) {
                    return -1;
                }
            }
            timer_step.stop();
            ROS_INFO_STREAM("Trajectory Optimization runtime: " << timer_step.elapsedSeconds());
            
            timer_total.stop();
            ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

            // Plot Planning Result
            resultPublisher_obj.reset(new ResultPublisher(nh, MPCPlanner_obj, corridor_obj, initTrajPlanner_obj, mission, param));
            resultPublisher_obj->plot(param.log);

            start_time = ros::Time::now().toSec();
            has_path = true;
        }
        if(has_path) {
            // Publish Swarm Trajectory
            current_time = ros::Time::now().toSec() - start_time;
            resultPublisher_obj.get()->update(current_time);
            resultPublisher_obj.get()->publish();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}