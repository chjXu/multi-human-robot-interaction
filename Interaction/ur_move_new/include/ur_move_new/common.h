#pragma once
#include <thread>
#include <cmath>
#include <queue>
#include <vector>
#include <mutex>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ur_move_new/collsion_avoidance.h>
#include <ur_move_new/action_follow.h>
#include <ur_move_new/grapper.h>

using namespace std;
using std::queue;
using std::vector;

class Human{
public:
    enum MODE
    {
        nothing = 0,
        avoidance,
        remote,
        point
    };
    

public:
    Human(int id, string identity, string mode):_id(id), _identity(identity){
        if(mode == "nothing"){
            state = this->nothing;
        }
        else if(mode == "avoidance"){
            state = this->avoidance;
        }
        else if(mode == "remote"){
            state = this->remote;
        }
        else if(mode == "point"){
            state = this->point;
        }
        else{
            cout << "Please enter right state." << endl;
        }

        tf_listener = new tf::TransformListener;
        coll_avoidance = new Collsion;
        action = new Action;
        grapper = new Grapper;
    }
    ~Human(){
        delete tf_listener;
        delete coll_avoidance;
        delete action;
        delete grapper;
    }

    void init();
    void run();
    void doNothing();
    bool updateJoint();

private:
    std::mutex lock;
    int _id;
    string _identity;

    std::thread multi_thread;
    MODE state;

    tf::TransformListener *tf_listener;
    tf::StampedTransform tool_rWrist;
    tf::StampedTransform tool_rElbow;
    tf::StampedTransform tool_rShoulder;
    tf::StampedTransform tool_lWrist;
    tf::StampedTransform tool_lElbow;
    tf::StampedTransform tool_lShoulder;
    tf::StampedTransform tool_obstance;
    vector<tf::StampedTransform> joint_tf;

    Collsion *coll_avoidance;
    Action *action;
    Grapper *grapper;
};