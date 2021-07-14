#include <iostream>
#include <ur_move_script/common.h>
#include <ctime>

using namespace std;

void Modules::run(){

    while(ros::ok()){ 
        robot->track();   //追踪marker，并达到夹取点  mode1 --> mode2
        sleep(0.5);

        if(robot->getModeState() == mode2 && robot->isToPlace()){
            gripper->clamp();
            sleep(0.5);
        }
        else if(robot->getModeState() == mode2 && !(robot->isToPlace())){
            gripper->loose();
            sleep(0.5);
        }


        // gripper->clamp(); //夹取
        // sleep(0.5);
        // robot->track();   //到放置点  toPlace:true --> false
        // sleep(2);
        // gripper->loose(); //松开
        // sleep(0.5);
        // robot->track();   //到初始点  toPlace:false --> true  ; mode2 --> mode1
        // sleep(0.5);
        // cout<<"have finished the work *******************************"<<endl;
        // break;
    }
}


int main(int argc , char** argv)
{
    ros::init(argc,argv,"track_grap"); 
    ros::NodeHandle nh("~");
    ros::Rate rate(30);

    Modules modules;
    modules.run();

    return 0;
}
