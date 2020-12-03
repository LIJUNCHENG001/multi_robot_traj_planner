#pragma once
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <fstream>
#include <ros/ros.h>
#include <string>
#include <random>
#include <chrono>

#include <iostream>
#include <exception>

using namespace rapidjson;

std::array<std::array<int,2>,16> inside{};
std::array<std::array<int,2>,16> outside{};

namespace SwarmPlanning {
    class Mission {
    public:
        int qn; // the number of robots
        std::vector<std::vector<double>> startState, goalState, max_vel, max_acc;
        std::vector<double> quad_size, quad_speed;

        bool setMission(const ros::NodeHandle &nh);
        void applyNoise(double max_noise);
    };


    bool Mission::setMission(const ros::NodeHandle &nh) {
        std::string mission_addr;
        nh.param<std::string>("mission", mission_addr, "demo");

        for (int i=0;i<16;i++)
        {
            if (i%4==0)
                inside[i][0]=-3;
            else if (i%4==1)
                inside[i][0]=-2;
            else if (i%4==2)
                inside[i][0]=2;
            else 
                inside[i][0]=3;

            if (i<4)
                inside[i][1]=3;
            else if(i<8)
                inside[i][1]=1;
            else if (i<12)
                inside[i][1]=-1;
            else 
                inside[i][1]=-3;
        }

        for (int i=0;i<16;i++)
        {
            if (i%8==0)
                outside[i][0]=-4;
            else if (i%8==1)
                outside[i][0]=-3;
            else if (i%8==2)
                outside[i][0]=-2;
            else if (i%8==3)
                outside[i][0]=-1;
            else if (i%8==4)
                outside[i][0]=1;
            else if (i%8==5)
                outside[i][0]=2;
            else if (i%8==6)
                outside[i][0]=3;
            else if (i%8==7)
                outside[i][0]=4;

            if (i<8)
                outside[i][1]=4;
            else 
                outside[i][1]=-4;
        }

        std::ifstream ifs(mission_addr);
        IStreamWrapper isw(ifs);
        Document document;
        if(document.ParseStream(isw).HasParseError()){
            ROS_ERROR_STREAM("There is no such mission file " << mission_addr << "\n");
            return false;
        }

        
        std::string seed_name="seed";

        Value::MemberIterator quadrotor = document["robots"].FindMember(seed_name.c_str());

        Value& maxVel = quadrotor->value.GetObject()["seed"];
        unsigned seed = maxVel[0].GetDouble();
 
        shuffle (inside.begin(), inside.end(), std::default_random_engine(seed));
    
        seed = seed+1;
        shuffle (outside.begin(), outside.end(), std::default_random_engine(seed));


        const Value& agents = document["agents"];

        qn = agents.Size();

        startState.resize(qn);
        goalState.resize(qn);
        quad_size.resize(qn);
        quad_speed.resize(qn);
        max_vel.resize(qn);
        max_acc.resize(qn);

        for(SizeType qi = 0; qi < agents.Size(); qi++){
            // name
            std::string name = agents[qi].GetObject()["name"].GetString();

            // start
            std::vector<double> state(9, 0);
            if (name=="robot")
            {
                const Value& start = agents[qi].GetObject()["start"];
                for(SizeType i = 0; i < start.Size(); i++){
                    state[i] = start[i].GetDouble();
                }
            }
            else if (name=="outside")
            {
                for(SizeType i = 0; i < 2; i++){
                    state[i] = outside[qi][i];
                }
            }
            else if (name=="inside")
            {
                for(SizeType i = 0; i < 2; i++){
                    state[i] = inside[15-(qn-qi-1)][i];
                }
            }
            startState[qi] = state;

            // goal
            state.assign(9, 0);
            if (name=="robot")
            {
                const Value& goal = agents[qi].GetObject()["goal"];
                for(SizeType i = 0; i < goal.Size(); i++){
                    state[i] = goal[i].GetDouble();
                }
            }
            else if (name=="outside")
            {
                for(SizeType i = 0; i < 2; i++){
                    state[i] = inside[qi][i];
                }
            }
            else if (name=="inside")
            {
                for(SizeType i = 0; i < 2; i++){
                    state[i] = outside[15-(qn-qi-1)][i];
                }
            }
            goalState[qi] = state;

            // radius
            quad_size[qi] = agents[qi].GetObject()["radius"].GetDouble();

            // speed
            quad_speed[qi] = agents[qi].GetObject()["speed"].GetDouble();

        }

        return true;
    }

    void Mission::applyNoise(double max_noise){
        srand((unsigned int)time(nullptr));
        for(int qi = 0; qi < qn; qi++) {
            for(int k = 0; k < 3; k++){
                startState[qi][k] += rand()/(double)RAND_MAX * max_noise;
                goalState[qi][k] += rand()/(double)RAND_MAX * max_noise;
            }
        }
    }
}
