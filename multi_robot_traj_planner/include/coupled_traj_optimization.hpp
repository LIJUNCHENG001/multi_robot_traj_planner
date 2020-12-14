#pragma once

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>

// ROS
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>


// Submodules
#include <corridor.hpp>
#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>

#include "com_fun.h"
#include "optimization.h"
#include "time.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

using CppAD::AD;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start;
size_t y_start;
size_t theta_start;
size_t v_start;
size_t omega_start;

std::vector<ros::Publisher> way_point_pub;

std::vector< std::array<int, 3> > relative_pair;
std::vector< int > agent_col_num;

class FG_eval {
     public:

      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;


      void operator()(ADvector& fg, const ADvector& vars) {
        // MPC implementation
        // fg a vector of constraints, x is a vector of constraints.
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        fg[0] = 0; 
        for (int qi = 0; qi < qn; qi ++){
            for (int i = 1; i < N; i++) {
                fg[0] += W_X * CppAD::pow(vars[x_start + N * qi + i] - plan[qi][i][0], 2);
                fg[0] += W_Y * CppAD::pow(vars[y_start + N * qi + i] - plan[qi][i][1], 2); 
            }
        }

        // Minimize the value gap between sequential actuations.
        for (int qi = 0; qi < qn; qi ++){
            for (int i = 0; i < N - 2; i++) {
              fg[0] += W_DV * CppAD::pow(vars[v_start + (N-1)* qi + i + 1] - vars[v_start + (N-1) * qi + i], 2);
              fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start + (N-1) * qi + i + 1] - vars[omega_start + (N-1) * qi + i], 2);
          }
        }

        // Initial constraints
        for (int qi = 0; qi < qn; qi ++){
            fg[1 + N * qi + x_start] = vars[x_start + N * qi];
            fg[1 + N * qi + y_start] = vars[y_start + N * qi];
            fg[1 + N * qi + theta_start] = vars[theta_start + N * qi];
        }

        // The rest of the constraints
        for (int qi = 0; qi < qn; qi ++){
            for (int i = 0; i < N - 1; i++) {
              // The state at time t+1 .
              AD<double> x1 = vars[x_start + N * qi + i + 1];
              AD<double> y1 = vars[y_start + N * qi + i + 1];
              AD<double> theta1 = vars[theta_start + N * qi + i + 1];

              // The state at time t.
              AD<double> x0 = vars[x_start + N * qi + i];
              AD<double> y0 = vars[y_start + N * qi + i];
              AD<double> theta0 = vars[theta_start + N * qi + i];

              // Only consider the actuation at time t.
              AD<double> v0 = vars[v_start + (N-1) * qi + i];
              AD<double> omega0 = vars[omega_start + (N-1) * qi + i];

              fg[2 + x_start + N * qi + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * DT);
              fg[2 + y_start + N * qi + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * DT);
              fg[2 + theta_start + N * qi + i] = theta1 - (theta0 + omega0 * DT);

            }
        }
        
        //std::cout<<relative_pair.size()<<std::endl;

        for (int j=0;j<qn;j++){
          int count=0;
          for(int i=0; i<relative_pair.size();i++)
          {
            if (relative_pair[i].at(1)==j)
              count++;
            else if (relative_pair[i].at(2)==j)
              count++;
          }
          agent_col_num.emplace_back(count);
        }

        int cont=N*3*qn+1;
        for (int i=0;i<relative_pair.size();i++)
        {
          fg[cont]=CppAD::pow(vars[x_start+N*relative_pair[i].at(1)+relative_pair[i].at(0)]-vars[x_start+N*relative_pair[i].at(2)+relative_pair[i].at(0)],2)
                  +CppAD::pow(vars[y_start+N*relative_pair[i].at(1)+relative_pair[i].at(0)]-vars[y_start+N*relative_pair[i].at(2)+relative_pair[i].at(0)],2);
          cont++;
        }       
    }
};


class MPCPlanner {
public:
    std_msgs::Float64MultiArray msgs_traj_info;
    std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;

    MPCPlanner(std::shared_ptr<Corridor> _corridor_obj,
               std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
               SwarmPlanning::Mission _mission,
               SwarmPlanning::Param _param)
            : corridor_obj(std::move(_corridor_obj)),
              initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))

    {
        M = initTrajPlanner_obj.get()->T.size()-1; // the number of segments

        outdim = 3; // the number of outputs (x,y,z)

        T = initTrajPlanner_obj.get()->T;
        initTraj = initTrajPlanner_obj.get()->initTraj;
        SFC = corridor_obj.get()->SFC;

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

        for (int qi=0; qi<qn; qi++)
        {
          for (int next=5;next<N;next+=5){
            if ((plan[qi][next][1]==plan[qi][0][1]) &&(plan[qi][next][0]==plan[qi][0][0]))
              continue;
            else{
              plan[qi][0][2]=atan2(plan[qi][next][1]-plan[qi][0][1],plan[qi][next][0]-plan[qi][0][0]);
              break;
            }
        }
      }

      double angle;

      for (i=0;i<qn;i++){
        for(j=0;j<M;j++){

          dx=initTraj[i][j+1].x()-initTraj[i][j].x();
          dy=initTraj[i][j+1].y()-initTraj[i][j].y();
          dis= dx*dx+dy*dy;
          
          if (j>0){
            if (dis>0.1){
              angle=atan2(dy,dx);
              if (angle-plan[i][5*(j-1)][2]>PI)
                angle=angle-2*PI;
              else if(plan[i][5*(j-1)][2]-angle>PI)
                angle=angle+2*PI;
            }
              else angle =plan[i][5*(j-1)][2];
          }
          else {angle=plan[i][0][2];}

          for (k=0;k<5;k++){
            plan[i][5*j+k][2]=angle;
          }

          
        }
      }

      for (i=0;i<qn;i++){
        for(j=0;j<M;j++){
          for (k=0;k<5;k++){
              plan[i][5*j+k][4]=(plan[i][5*(j+1)][2]-plan[i][5*j][2])*0.2/DT;
            }
          }
      }

      x_start = 0;
      y_start = x_start + qn*N;
      theta_start = y_start + qn*N;
      v_start = theta_start + qn*N;
      omega_start = v_start + qn*(N - 1);


      std::array<int, 3> sa;
      for (k=0;k<N;k++){
        for(i=0;i<qn;i++){
          for (j=i+1;j<qn;j++)
          {
            dis= pow(plan[i][k][0]-plan[j][k][0],2.0)+pow(plan[i][k][1]-plan[j][k][1],2.0);
            if (dis<1.3)
              relative_pair.emplace_back(sa={k, i, j});
          }
        }
      }

    }

    void Solve() {
      bool ok = true;
      size_t i;
      typedef CPPAD_TESTVECTOR(double) Dvector;


      // Set the number of model variables (includes both states and inputs).
      // For example: If the state is a 4 element vector, the actuators is a 2
      // element vector and there are 10 timesteps. The number of variables is:
      size_t n_vars = (N * 3 + (N - 1) * 2) * qn;
      // Set the number of constraints
      size_t n_constraints = N * 3 * qn;
      //size_t all_constraints = n_constraints + qn*(qn-1)*int(N/2)/2;
      size_t all_constraints = n_constraints + relative_pair.size();

      // Initial value of the independent variables.
      // SHOULD BE 0 besides initial state.
      Dvector vars(n_vars);

      Dvector vars_lowerbound(n_vars);
      Dvector vars_upperbound(n_vars);
      // Set lower and upper limits for variables.
      for (int i = 0; i < v_start; i++) {
        vars_lowerbound[i] = -BOUND;
        vars_upperbound[i] = BOUND;
      }


      for (int qi=0;qi<qn;qi++){

        int count=0;

        for (int i = 0; i < N; i++) {
          vars[x_start+qi*N+i]=plan[qi][i][0];
          vars[y_start+qi*N+i]=plan[qi][i][1];
          vars[theta_start+qi*N+i]=plan[qi][i][2];
          if (i<N-1)
          {vars[v_start+qi*(N-1)+i]=plan[qi][i][3];
           vars[omega_start+qi*(N-1)+i]=plan[qi][i][4];}
          
          
          if ( i>1 && i==SFC[qi][count+1].second && count<SFC[qi].size()-1){
            count++;
          }

          if (plan[qi][i][0]-SFC[qi][count].first[0]>2)
            vars_lowerbound[x_start+qi*N+i]=plan[qi][i][0]-2;
          else
            vars_lowerbound[x_start+qi*N+i]=SFC[qi][count].first[0];
          if (SFC[qi][count].first[3]-plan[qi][i][0]>2)
            vars_upperbound[x_start+qi*N+i]=plan[qi][i][0]+2;
          else
            vars_upperbound[x_start+qi*N+i]=SFC[qi][count].first[3];
          if (plan[qi][i][1]-SFC[qi][count].first[1]>2)
            vars_lowerbound[y_start+qi*N+i]=plan[qi][i][1]-2;
          else
            vars_lowerbound[y_start+qi*N+i]=SFC[qi][count].first[1];
          if (SFC[qi][count].first[4]-plan[qi][i][1]>2)
            vars_upperbound[y_start+qi*N+i]=plan[qi][i][1]+2;
          else
            vars_upperbound[y_start+qi*N+i]=SFC[qi][count].first[4]; 
        }
      }


      for (int i = v_start; i < omega_start; i++) {
        vars_lowerbound[i] = 0;
        vars_upperbound[i] = MAXV;
        if (param.backward_enable)
        vars_lowerbound[i] = -MAXV;
      }

      for (int i = omega_start; i < n_vars; i++) {
        vars_lowerbound[i] = -MAXOMEGA;
        vars_upperbound[i] = MAXOMEGA;
      }


      // Lower and upper limits for the constraints
      // Should be 0 besides initial state.
      Dvector constraints_lowerbound(all_constraints);
      Dvector constraints_upperbound(all_constraints);
      for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
      }

      double inter_collision=2*mission.quad_size[0]+0.2;
      inter_collision=inter_collision*inter_collision;
      for (int i= n_constraints;i<all_constraints;i++)
      {
        constraints_lowerbound[i] = inter_collision;
        constraints_upperbound[i] = BOUND;
      }

      for (int qi = 0; qi < qn; qi ++){
          double startangle;
      for (int next=5;next<N;next+=5){
        if ((plan[qi][next][1]==plan[qi][0][1]) &(plan[qi][next][0]==plan[qi][0][0]))
          continue;
        else{
          startangle=atan2(plan[qi][next][1]-plan[qi][0][1],plan[qi][next][0]-plan[qi][0][0]);
          break;
        }
      }

          constraints_lowerbound[x_start + N * qi] = plan[qi][0][0];
          constraints_lowerbound[y_start + N * qi] = plan[qi][0][1];
          constraints_upperbound[x_start + N * qi] = plan[qi][0][0];
          constraints_upperbound[y_start + N * qi] = plan[qi][0][1];

          if(param.initial_angle)
          {
            constraints_lowerbound[theta_start + N * qi] = mission.startState[qi][2]*PI/2;
            constraints_upperbound[theta_start + N * qi] = mission.startState[qi][2]*PI/2;
          }
          else
          {
            constraints_lowerbound[theta_start + N * qi] = startangle;
            constraints_upperbound[theta_start + N * qi] = startangle;
          }
      }


      // object that computes objective and constraints
      FG_eval fg_eval;
      // options for IPOPT solver
      std::string options;

      options += "Numeric tol          1e-5\n";
      options += "String linear_solver mumps\n";
      // Uncomment this if you'd like more print information
      options += "Integer print_level  1\n";
      // NOTE: Setting sparse to true allows the solver to take advantage
      // of sparse routines, this makes the computation MUCH FASTER. If you
      // can uncomment 1 of these and see if it makes a difference or not but
      // if you uncomment both the computation time should go up in orders of
      // magnitude.
      options += "Sparse  true        forward\n";
      options += "Sparse  true        reverse\n";
      // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
      // Change this as you see fit.

      // place to return solution
      CppAD::ipopt::solve_result<Dvector> solution;


      // solve the problem
      CppAD::ipopt::solve<Dvector, FG_eval>(
          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
          constraints_upperbound, fg_eval, solution);

      // Check some of the solution values
      ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
      if (ok)
      {
        ROS_INFO_STREAM("Optimization Success!");
        ROS_INFO_STREAM("Cost: "<<solution.obj_value);
      }
      


      for (int qi=0;qi<qn;qi++){
        for (int i=0;i<N;i++){
            plan[qi][i][0]=solution.x[x_start+qi*N+i];
            plan[qi][i][1]=solution.x[y_start+qi*N+i];
            plan[qi][i][2]=solution.x[theta_start+qi*N+i];
            plan[qi][i][3]=solution.x[v_start+qi*(N-1)+i];
            plan[qi][i][4]=solution.x[omega_start+qi*(N-1)+i];
        }
      }
    }

    bool update(bool log){

        int qi=0;

        nav_msgs::Path path;
        geometry_msgs::PoseStamped pp;

        path.header.frame_id="world";
        path.header.stamp=ros::Time::now();


        Solve();

        for (int qi=0;qi<qn;qi++)
        {
          nav_msgs::Path path;
          path.header.frame_id="map";
          path.header.stamp=ros::Time::now();
          for (int i=0;i<N;i++)
          {
              pp.pose.position.x=plan[qi][i][0];
              pp.pose.position.y=plan[qi][i][1];
              pp.pose.orientation=tf::createQuaternionMsgFromYaw(plan[qi][i][2]);
              path.poses.push_back(pp);

              if (fabs(plan[qi][i][0]-plan[qi][N-1][0])<0.01){
                  if(fabs(plan[qi][i][1]-plan[qi][N-1][1])<0.01){
                      break;
                  }
              }
          }
        way_point_pub[qi].publish(path);
      }

        return true;
    }

private:
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    initTraj_t initTraj;
    std::vector<double> T;
    SFC_t SFC;

    int M, phi, outdim;

    // std::shared_ptr<Eigen::MatrixXd> Q_obj, Aeq_obj, Alq_obj, deq_obj, dlq_obj;
    Eigen::MatrixXd Q_base, Aeq_base, Alq, deq, dlq, basis;
    Eigen::MatrixXd dummy;
    std::vector<Eigen::MatrixXd> coef;


    void createMsg(){
        std::vector<double> traj_info;
        traj_info.emplace_back(N);
        traj_info.emplace_back(qn);
        traj_info.insert(traj_info.end(), T.begin(), T.end());
        msgs_traj_info.data = traj_info;

        msgs_traj_coef.resize(N);
        for(int qi = 0; qi < N; qi++) {
            std_msgs::MultiArrayDimension rows;
            rows.size = M * (qn + 1);
            msgs_traj_coef[qi].layout.dim.emplace_back(rows);

            std_msgs::MultiArrayDimension cols;
            cols.size = outdim;
            msgs_traj_coef[qi].layout.dim.emplace_back(cols);

            std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
            msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
        }
    }
};
