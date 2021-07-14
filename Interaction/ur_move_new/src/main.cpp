#include <ur_move_new/common.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

void Human::doNothing(){

}

bool Human::updateJoint(){
    while(ros::ok()){
        try
        {
            ROS_INFO("listenering...");
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/rWrist", ros::Time(0), ros::Duration(3.0));
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/rElbow", ros::Time(0), ros::Duration(3.0));
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/rShoulder", ros::Time(0), ros::Duration(3.0));
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/lWrist", ros::Time(0), ros::Duration(3.0));
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/lElbow", ros::Time(0), ros::Duration(3.0));
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/lShoulder", ros::Time(0), ros::Duration(3.0));
    
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/rWrist", ros::Time(0), tool_rWrist);
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/rElbow", ros::Time(0), tool_rElbow);
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/rShoulder", ros::Time(0), tool_rShoulder);
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/lWrist", ros::Time(0), tool_lWrist);
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/lElbow", ros::Time(0), tool_lElbow);
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/lShoulder", ros::Time(0), tool_lShoulder);

            break;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        return true;
    }
    
}

void Human::init(){
    while(ros::ok()){
        try
        {
            cout << "wait for connecting" << endl;
            tf_listener->waitForTransform("tool0", "human_" + to_string(this->_id) + "/rWrist", ros::Time(0), ros::Duration(1.0));
            tf_listener->lookupTransform("tool0", "human_" + to_string(this->_id) + "/rWrist", ros::Time(0), tool_obstance);
            break;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }   
    }
}

void Human::run(){
    // switch (this->state)
    // {
    // case avoidance:
    //     ROS_INFO("Avoidancing.");
    //     //init();
    //     //while(ros::ok()){
    //         if(updateJoint()){
    //             this->joint_tf.clear();
    //             this->joint_tf.emplace_back(tool_lWrist);
    //             this->joint_tf.emplace_back(tool_lElbow);
    //             this->joint_tf.emplace_back(tool_lShoulder);
    //             this->joint_tf.emplace_back(tool_rWrist);
    //             this->joint_tf.emplace_back(tool_rElbow);
    //             this->joint_tf.emplace_back(tool_rShoulder);
    //         }
    //         coll_avoidance->setTest(tool_obstance);
    //         coll_avoidance->collsion_avoidance(joint_tf, this->_id);
    //     //}
    // case remote:
    //     action->follow_hand();
    //     break;
    // case point:
    //     grapper->grapper();
    // default:
    //     break;
    // }

    // if(this->state == nothing){
    //     doNothing();
    // }
    if(this->state == avoidance){
        // 状态读取正常
        if(updateJoint()){
                this->joint_tf.clear();
                this->joint_tf.emplace_back(tool_lWrist);
                this->joint_tf.emplace_back(tool_lElbow);
                this->joint_tf.emplace_back(tool_lShoulder);
                this->joint_tf.emplace_back(tool_rWrist);
                this->joint_tf.emplace_back(tool_rElbow);
                this->joint_tf.emplace_back(tool_rShoulder);
            }
            coll_avoidance->setTest(tool_obstance);
            coll_avoidance->collsion_avoidance(joint_tf, this->_id);
    }
    else if(this->state == remote){
        ROS_INFO("Remote.");
        action->follow_hand();
    }
    // else if(this->state == point && this->_identity == "operator"){
    //     //grapper();
    // }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "human_robot_interaction");
    ros::NodeHandle nh("~");
    ros::Rate rate(30);

    string operator_state = "avoidance";
    string non_operator_state = "avoidance";

    //nh.getParam("operator", operator_state);
    //nh.getParam("non_operator", non_operator_state);

    //Human *human_operator = new Human(0, "operator", operator_state);
    Human *human_non_operator = new Human(1, "non_operator", non_operator_state);

    //human_operator->init();
    human_non_operator->init();

    ROS_INFO("Init finish...");

    while(ros::ok()){
        //human_operator->run();
        human_non_operator->run();

        ros::spinOnce();
        rate.sleep();
    }
}