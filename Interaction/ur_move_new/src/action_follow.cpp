#include<ur_move_new/action_follow.h>
#include <ctime>

void Action::init(){
    this->desired_goal_list.clear();
    for (int i = 0; i < 5; i++)
    {
        try
        {
            //tf_listener.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));
            //tf_listener.lookupTransform("base", "tool0", ros::Time(0), base_tool);
            //tf_listener.waitForTransform("base", "target_" + to_string(i), ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("base", "target_" + to_string(i), ros::Time(0), rWrist_order);
            desired_goal_list.push_back(rWrist_order);
        }
        catch(tf::TransformException ex)
        {
            
        }
    }

    // tf::Vector3 offset(base_tool.getOrigin().x() - rWrist_order.getOrigin().x(),
    //           base_tool.getOrigin().y() - rWrist_order.getOrigin().y(),
    //           base_tool.getOrigin().z() - rWrist_order.getOrigin().z());

    // this->offset = offset;
    ROS_INFO("Finish init.");
}

void Action::follow_hand(){
    ROS_INFO("Waitting");
    //while (ros::ok())
    //{
        init();
        // try
        // {
        //     tf_listener.waitForTransform("base", "human_0/rWrist", ros::Time(0), ros::Duration(3.0));
        //     tf_listener.lookupTransform("base", "human_0/rWrist", ros::Time(0), rWrist_order);
        //     tf_listener.waitForTransform("marker_0", "imu", ros::Time(0), ros::Duration(3.0));
        //     tf_listener.lookupTransform("marker_0", "imu", ros::Time(0), rWrist_imu);
        // }
        // catch(tf::TransformException ex)
        // {
        //     continue;
        // }
        cout << desired_goal_list.size() << endl;
        for (size_t i = 0; i < desired_goal_list.size(); i++)
        {
            desired_position.setOrigin(tf::Vector3(desired_goal_list[i].getOrigin().x(),
                                               desired_goal_list[i].getOrigin().y(),
                                               desired_goal_list[i].getOrigin().z()));
            desired_position.setRotation(desired_goal_list[i].getRotation());
            br.sendTransform(tf::StampedTransform(desired_position, ros::Time::now(), "base", "attraction_goal"));        
        }
        

        // desired_position.setOrigin(tf::Vector3(rWrist_order.getOrigin().x() + this->offset.x(),
        //                                        rWrist_order.getOrigin().y() + this->offset.y(),
        //                                        rWrist_order.getOrigin().z() + this->offset.z()));
        // desired_position.setRotation(rWrist_imu.getRotation());
        // br.sendTransform(tf::StampedTransform(desired_position, ros::Time::now(), "base", "desired_goal"));        
    //    ros::spinOnce();
    //}
}