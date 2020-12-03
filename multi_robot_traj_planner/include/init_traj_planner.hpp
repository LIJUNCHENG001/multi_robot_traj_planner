#pragma once

#include <sp_const.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <param.hpp>
#include <mission.hpp>

class InitTrajPlanner {
public:
    initTraj_t initTraj; // discrete initial trajectory: pi_0,...,pi_M
    std::vector<double> T; // segment time: T_0,...,T_M

    virtual bool update(bool log) = 0;

    InitTrajPlanner(std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
                    SwarmPlanning::Mission _mission,
                    SwarmPlanning::Param _param)
            : distmap_obj(std::move(_distmap_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        grid_x_min = ceil((param.world_x_min + SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_y_min = ceil((param.world_y_min + SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_z_min = ceil((param.world_z_min + SP_EPSILON) / param.grid_z_res) * param.grid_z_res;

        grid_x_max = floor((param.world_x_max - SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_y_max = floor((param.world_y_max - SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_z_max = floor((param.world_z_max - SP_EPSILON) / param.grid_z_res) * param.grid_z_res;

        dimx = (int)round((grid_x_max - grid_x_min) / param.grid_xy_res) + 1;
        dimy = (int)round((grid_y_max - grid_y_min) / param.grid_xy_res) + 1;
        dimz = (int)round((grid_z_max - grid_z_min) / param.grid_z_res) + 1;
    }

protected:
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    double grid_x_min, grid_y_min, grid_z_min, grid_x_max, grid_y_max, grid_z_max;
    int dimx, dimy, dimz;
};