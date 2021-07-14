#include <ur_move_script/gripper.h>

void Gripper::clamp()
{
    srv_move.request.position = 20;
    srv_move.request.speed = 100;
    srv_move.request.acceleration = 50;
    srv_move.request.torque = 1;
    srv_move.request.tolerance = 100;
    srv_move.request.waitFlag = true;
    std::cout<<"进行抓取........"<<std::endl;
    move_to_client.call(srv_move);

}

void Gripper::loose()
{
    srv_move.request.position = 80;
	srv_move.request.speed = 100;
	srv_move.request.acceleration = 50;
	srv_move.request.torque = 1;
	srv_move.request.tolerance = 100;
	srv_move.request.waitFlag = true;
    std::cout<<"放下.........."<<std::endl;
	move_to_client.call(srv_move);
}

// void Gripper::init()
// {
//     tf::Quaternion q(0,0,0,1.0);
//     place_point.setOrigin(tf::Vector3(-0.5,-0.5,0.15));
//     place_point.setRotation(q);

//     init_point.setOrigin(tf::Vector3(0,0,0.5));
//     init_point.setRotation(q);
// }
