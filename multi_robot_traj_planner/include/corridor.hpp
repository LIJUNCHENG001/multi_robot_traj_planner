#pragma once

#include <Eigen/Dense>

#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>
#include <timer.hpp>
#include <optimization.h>

// Set the timestep length
size_t N;
int qn;

double plan[32][500][5]; 

class Corridor {
public:
    SFC_t SFC; // safe flight corridors to avoid obstacles

    Corridor(std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
             std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
             SwarmPlanning::Mission _mission,
             SwarmPlanning::Param _param)
            : initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              distmap_obj(std::move(_distmap_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        initTraj = initTrajPlanner_obj.get()->initTraj;
        T = initTrajPlanner_obj.get()->T;
        makespan = T.back();
        M = T.size()-1; // the number of segments

        qn = mission.qn; // the number of agents

        int i,j,k;

        double dx;
        double dy;
        double dis;
        

        for (i=0;i<qn;i++){
            for(j=0;j<M;j++){

                dx=(initTraj[i][j+1].x()-initTraj[i][j].x())/5;
                dy=(initTraj[i][j+1].y()-initTraj[i][j].y())/5;
                dis= 25*(dx*dx+dy*dy);
                

                for (k=0;k<5;k++){
                    plan[i][5*j+k][0]=initTraj[i][j].x()+dx*k;
                    plan[i][5*j+k][1]=initTraj[i][j].y()+dy*k;

                    if (dis<0.1)
                    {
                      plan[i][5*j+k][3]=0;
                    }
                    if (dis<1.1){
                      plan[i][5*j+k][3]=MAXV*0.7;
                    }
                    else{
                      plan[i][5*j+k][3]=MAXV;
                    }
                }
            }
            plan[i][5*M][0]=initTraj[i][M].x();
            plan[i][5*M][1]=initTraj[i][M].y();
        }

        N = M*5+1;
    }

    bool update(bool log){
        return updateObsBox(log);
    }

private:
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    initTraj_t initTraj;
    std::vector<double> T;
    double makespan;
    int M;

    bool isObstacleInBox(const std::vector<double>& box, double margin){
        double x,y,z;
        int count1 = 0;
        for(double i = box[0]; i < box[3] + SP_EPSILON_FLOAT; i += param.box_xy_res) {
            int count2 = 0;
            for(double j = box[1]; j < box[4] + SP_EPSILON_FLOAT; j += param.box_xy_res) {
                int count3 = 0;
                for(double k = box[2]; k < box[5] + SP_EPSILON_FLOAT; k += param.box_z_res) {
                    x = i + SP_EPSILON_FLOAT;
                    if(count1 == 0)
                        x = box[0] - SP_EPSILON_FLOAT;
                    y = j + SP_EPSILON_FLOAT;
                    if(count2 == 0)
                        y = box[1] - SP_EPSILON_FLOAT;
                    z = k + SP_EPSILON_FLOAT;
                    if(count3 == 0)
                        z = box[2] - SP_EPSILON_FLOAT;

                    octomap::point3d cur_point(x, y, z);
                    float dist = distmap_obj.get()->getDistance(cur_point);

                    assert(dist>=0);
                    if(dist < margin - SP_EPSILON_FLOAT){                      
                        return true;
                    }
                    count3++;
                }
                count2++;
            }
            count1++;
        }

        return false;
    }

    bool isBoxInBoundary(const std::vector<double>& box){
        return box[0] > param.world_x_min - SP_EPSILON&&
               box[1] > param.world_y_min - SP_EPSILON&&
               box[3] < param.world_x_max + SP_EPSILON&&
               box[4] < param.world_y_max + SP_EPSILON;
    }

    bool isPointInBox(const octomap::point3d& point,
                      const std::vector<double>& box){
        return point.x() > box[0] - SP_EPSILON &&
               point.y() > box[1] - SP_EPSILON &&
               point.x() < box[3] + SP_EPSILON &&
               point.y() < box[4] + SP_EPSILON;
    }

    void expand_box(std::vector<double>& box, double margin) {
        std::vector<double> box_cand, box_update;
        std::vector<int> axis_cand{0, 1, 2, 3, 4, 5};

        int i = -1;
        int axis;
        while (!axis_cand.empty()) {
            box_cand = box;
            box_update = box;

            //check update_box only! update_box + current_box = cand_box
            while ( !isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update) ) {
                i++;
                if(i >= axis_cand.size()){
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                box = box_cand;
                box_update = box_cand;

                //expand cand_box and get updated part of box(update_box)
                if (axis < 3) {
                    box_update[axis+3] = box_cand[axis];
                    if (axis == 2){
                        box_cand[axis] = box_cand[axis] - param.box_z_res;
                    }
                    else {
                        box_cand[axis] = box_cand[axis] - param.box_xy_res;
                    }
                    box_update[axis] = box_cand[axis];
                }
                else{
                    box_update[axis-3] = box_cand[axis];
                    if (axis == 5){
                        box_cand[axis] = box_cand[axis] + param.box_z_res;
                    }
                    else{
                        box_cand[axis] = box_cand[axis] + param.box_xy_res;
                    }
                    box_update[axis] = box_cand[axis];
                }
            }
            axis_cand.erase(axis_cand.begin() + i);
            if(i > 0) {
                i--;
            }
            else{
                i = axis_cand.size()-1;
            }
        }
    }

    bool updateObsBox(bool log){
        double x_next, y_next, z_next, dx, dy, dz;

        int count=0;
        SFC.resize(mission.qn);
        for (size_t qi = 0; qi < mission.qn; ++qi) {
            std::vector<double> box_prev{0,0,0,0,0,0};

            double height=param.height;
            for (int i = 0; i < N-1; i++) {

                double x = plan[qi][i][0];
                double y = plan[qi][i][1];
                double z = height;

                std::vector<double> box;

                x_next = plan[qi][i+1][0];
                y_next = plan[qi][i+1][1];
                z_next = height;

                if(isPointInBox(octomap::point3d(x_next, y_next, z_next), box_prev)){
                    continue;
                }

                // Initialize box

                box.emplace_back(round(std::min(x,x_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::min(y,y_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::min(z,z_next) / param.box_z_res) * param.box_z_res);
                box.emplace_back(round(std::max(x,x_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::max(y,y_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::max(z,z_next) / param.box_z_res) * param.box_z_res);
                box.emplace_back(round(std::max(z,z_next) / param.box_z_res) * param.box_z_res + param.box_z_res);



                //if (isObstacleInBox(box, mission.quad_size[qi])) {
                if (isObstacleInBox(box, mission.quad_size[qi]+0.1)) {
                    box.clear();
                    box.emplace_back(round(x_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(y_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(z_next / param.box_z_res) * param.box_z_res);
                    box.emplace_back(round(x_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(y_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(z_next / param.box_z_res) * param.box_z_res);

                }
                expand_box(box, mission.quad_size[qi]+0.1);

                if (box[3]==box[0] || box[4]==box[1]){
                    box.emplace_back(round(x_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(y_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(z_next / param.box_z_res) * param.box_z_res);
                    box.emplace_back(round(x_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(y_next / param.box_xy_res) * param.box_xy_res);
                    box.emplace_back(round(z_next / param.box_z_res) * param.box_z_res);
                    expand_box(box, mission.quad_size[qi]+0.05);
                    count++;
                }

                SFC[qi].emplace_back(std::make_pair(box, i+1));

                box_prev = box;
            }

        }

        if(param.log)
            std::cout<<"Safe Corridor construction error: "<<count<<std::endl;

        return true;
    }

};
