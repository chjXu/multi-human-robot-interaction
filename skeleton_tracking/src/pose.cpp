#include <human_pose/pose.h>

using namespace HumanPose;

void Pose_Info::setPose(human_pose_msgs::HumanList human_list, int id){
    // topic中的数据都是cm为单位的， in 1920*1080
    for(int i=0; i<15; ++i){
        pose_3d[i].x = human_list.human_list[id].body_key_points_prob[i].x;
        pose_3d[i].y = human_list.human_list[id].body_key_points_prob[i].y;
        pose_3d[i].z = human_list.human_list[id].body_key_points_prob[i].z;
        pose_3d[i].p = human_list.human_list[id].body_key_points_prob[i].p;
        pose_3d[i].available = true;
    }
}

void Pose_Info::trangulation(Eigen::Matrix3d rot, Eigen::Matrix<double, 3, 1> trans){
    if(pose_3d.empty()){
        return;
    }

    for(int i=0; i<pose_3d.size(); ++i){
        Eigen::Matrix<double,3,1> world_point;
        Eigen::Matrix<double,3,1> cam_point; 

        cam_point(0,0) = pose_3d[i].x / 100.0;
        cam_point(1,0) = pose_3d[i].y / 100.0;
        cam_point(2,0) = pose_3d[i].z / 100.0;
        
        world_point = rot * cam_point + trans;

        pose_3d[i].x = world_point(0,0);
        pose_3d[i].y = world_point(1,0);
        pose_3d[i].z = world_point(2,0);
    }
}

void Pose_Info::setAction(int action){
    this->action = action;
}