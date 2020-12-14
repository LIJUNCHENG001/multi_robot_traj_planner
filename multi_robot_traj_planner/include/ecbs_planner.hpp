#pragma once

#include "init_traj_planner.hpp"
#include <environment.hpp>

using namespace libMultiRobotPlanning;

class ECBSPlanner : public InitTrajPlanner{
public:
    ECBSPlanner(std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
                SwarmPlanning::Mission _mission,
                SwarmPlanning::Param _param)
                : InitTrajPlanner(std::move(_distmap_obj),
                                  std::move(_mission),
                                  std::move(_param))
    {
        setObstacles();
        setWaypoints();
    }

    bool update(bool log) override {
        Environment mapf(dimx, dimy, dimz, ecbs_obstacles, ecbs_goalLocations, mission.quad_size, param.grid_xy_res);
        ECBS<State, Action, int, Conflict, Constraints, Environment> ecbs(mapf, param.ecbs_w);
        std::vector<PlanResult<State, Action, int>> solution;

        // Execute ECBS algorithm
        bool success = ecbs.search(ecbs_startStates, solution, param.log);
        if(!success){
            ROS_ERROR("ECBSPlanner: ECBS Failed!");
            return false;
        }

        // Update segment time
        int cost = 0;
        int makespan = 0;
        for (const auto &s : solution) {
            cost += s.cost;
            //makespan = std::max<int>(makespan, s.cost);
            makespan = std::max<int>(makespan, s.states.size());
        }
        for(int i = 0; i <= makespan + 2; i++){
            T.emplace_back(i * param.time_step);
        }
        if(log) {
            ROS_INFO_STREAM("ECBSPlanner: M=" << T.size() - 1 << ", makespan=" << T.back());
        }

        // Append start, goal points to both ends respectively
        initTraj.resize(solution.size());
        for (size_t a = 0; a < solution.size(); ++a) {
            initTraj[a].emplace_back(octomap::point3d(mission.startState[a][0],
                                                      mission.startState[a][1],
                                                      mission.startState[a][2]));

            for (const auto &state : solution[a].states) {
                initTraj[a].emplace_back(octomap::point3d(state.first.x * param.grid_xy_res + grid_x_min,
                                                          state.first.y * param.grid_xy_res + grid_y_min,
                                                          state.first.z));

            }
            while(initTraj[a].size() <= makespan + 2){
                initTraj[a].emplace_back(octomap::point3d(mission.goalState[a][0],
                                                          mission.goalState[a][1],
                                                          mission.goalState[a][2]));
            }
        }
        return true;
    }

private:
    std::unordered_set<Location> ecbs_obstacles;
    std::vector<State> ecbs_startStates;
    std::vector<Location> ecbs_goalLocations;

    // Find the location of obstacles in grid-space
    bool setObstacles(){
        double r = 0;
        for(int qi = 0; qi < mission.qn; qi++){
            if(r < mission.quad_size[qi]){
                r = mission.quad_size[qi];
            }
        }

        int x,y,z;
        for (double k = grid_z_min; k < grid_z_max + SP_EPSILON; k += param.grid_z_res ){
            for (double i = grid_x_min; i < grid_x_max + SP_EPSILON; i += param.grid_xy_res*0.2 ){
                for (double j = grid_y_min; j < grid_y_max + SP_EPSILON; j += param.grid_xy_res*0.2 ){
                    octomap::point3d cur_point(i,j,k);
                    float dist = distmap_obj.get()->getDistance(cur_point);
                    if(dist < 0){
                        return false;
                    }

                    // To prevent obstacles from putting between grid points, grid_margin is used
                    if (dist < r + param.grid_margin){
                        x = (int)round((i - grid_x_min) / (param.grid_xy_res*0.2));
                        y = (int)round((j - grid_y_min) / (param.grid_xy_res*0.2));
                        z = (int)round((k - grid_z_min) / param.grid_z_res);
                        ecbs_obstacles.insert(Location(x, y, z));
                    }
                }
            }
        }
        return true;
    }

    // Set start, goal points of ECBS
    bool setWaypoints(){
        int xig, yig, zig, xfg, yfg, zfg;
        for(int i = 0; i < mission.qn; i++){
            // For start, goal point of ECBS, we use the nearest grid point.
            xig = (int)round((mission.startState[i][0] - grid_x_min) / param.grid_xy_res);
            yig = (int)round((mission.startState[i][1] - grid_y_min) / param.grid_xy_res);
            zig = (int)round((mission.startState[i][2] - grid_z_min) / param.grid_z_res);
            xfg = (int)round((mission.goalState[i][0] - grid_x_min) / param.grid_xy_res);
            yfg = (int)round((mission.goalState[i][1] - grid_y_min) / param.grid_xy_res);
            zfg = (int)round((mission.goalState[i][2] - grid_z_min) / param.grid_z_res);

            if(ecbs_obstacles.find(Location(xig * 5 , yig * 5 , zig)) != ecbs_obstacles.end()){
                ROS_ERROR_STREAM("ECBSPlanner: start of agent " << i << " is occluded by obstacle");
                return false;
            }
            if(ecbs_obstacles.find(Location(xfg * 5, yfg * 5 , zfg)) != ecbs_obstacles.end()) {
                ROS_ERROR_STREAM("ECBSLauncher: goal of agent " << i << " is occluded by obstacle");
                return false;
            }

            if (param.initial_angle)
            {
                ecbs_startStates.emplace_back(State(0, xig, yig, mission.startState[i][2]));
                ecbs_goalLocations.emplace_back(Location(xfg, yfg, mission.goalState[i][2]));
            }
            else
            {
                ecbs_startStates.emplace_back(State(0, xig, yig, zig));
                ecbs_goalLocations.emplace_back(Location(xfg, yfg, zfg));
            }
        }
        return true;
    }
};
