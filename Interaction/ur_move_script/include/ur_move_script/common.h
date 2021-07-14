#pragma once
#include <iostream>
#include <ur_move_script/manipulator.h>
#include <ur_move_script/gripper.h>

using namespace std;

class Modules
{
public:
    Modules(){
        robot = new Manipulator;
        gripper = new Gripper;
    }
 
    void run();

    ~Modules(){
        delete robot;
        delete gripper;
    }

private:
    Manipulator *robot;
    Gripper *gripper;
};
