#include <ur_move_new/collsion_avoidance.h>

void Collsion::init(){
    // att_goal_one.setOrigin(tf::Vector3(-0.4, -0.5, 0.17));
    // att_goal_middle.setOrigin(tf::Vector3(0.0, -0.5, 0.17));
    // att_goal_two.setOrigin(tf::Vector3(0.4, -0.5, 0.17));

    // tf::Quaternion q(0.907, 0.418, -0.042, 0.013);
    // q = q.normalize();
    // att_goal_one.setRotation(q);
    // att_goal_middle.setRotation(q);
    // att_goal_two.setRotation(q);

    // att_goals.push_back(att_goal_one);
    // att_goals.push_back(att_goal_middle);
    // att_goals.push_back(att_goal_two);

    // target_queue.push(2);
    // // target_queue.push(1);
    // target_queue.push(0);
    // // target_queue.push(1);

    // this->goal_index = target_queue.front();

    // target_queue.pop();
    // target_queue.push(goal_index);

	goal1.setOrigin(tf::Vector3(0.139, -0.42, 0.200));
    goal2.setOrigin(tf::Vector3(0.227, -0.503, 0.2));
    goal3.setOrigin(tf::Vector3(-0.048, -0.56, 0.2));
    goal4.setOrigin(tf::Vector3(0.154, -0.586, 0.2));
    goal5.setOrigin(tf::Vector3(-0.055, -0.48, 0.2));
    goal6.setOrigin(tf::Vector3(0.069, -0.425, 0.4));
    place_goal.setOrigin(tf::Vector3(-0.6, -0.1, 0.300));

	//tf::Quaternion q(0.00023, 0.999, -0.022, -0.016);
    tf::Quaternion q(0.00698, 0.999948, 0000724, -0.0017);
    //tf::Quaternion q(0.0238, 0.99478, 0.0966, 0.0225);
    //tf::Quaternion q(0.375, 0.925, -0.011, 0.062);
	q = q.normalize();
	//q.setRPY(0.0, 0.0, 0.0);
	goal1.setRotation(q);
	goal2.setRotation(q);
	goal3.setRotation(q);
   	goal4.setRotation(q);
	goal5.setRotation(q);
    goal6.setRotation(q);
    place_goal.setRotation(q);
  
    att_goals.push_back(place_goal);
	att_goals.push_back(goal1);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
	att_goals.push_back(goal2);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
	att_goals.push_back(goal3);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
    att_goals.push_back(goal4);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
	att_goals.push_back(goal5);
    att_goals.push_back(goal6);
	
    target_queue.push(14);
    target_queue.push(13);
	target_queue.push(12);
	target_queue.push(11);
    target_queue.push(10);
    target_queue.push(9);
	target_queue.push(8);
	target_queue.push(7);
    target_queue.push(6);
	target_queue.push(5);
	target_queue.push(4);
	target_queue.push(3);
	target_queue.push(2);
    target_queue.push(1);
	target_queue.push(0);
	this->goal_index = target_queue.front();

	target_queue.pop();
	target_queue.push(goal_index);
}

void Collsion::setTest(tf::StampedTransform& test){
    this->test = test;
}

int Collsion::judgeJoint(vector<tf::StampedTransform>& joint_tf){
    int j = 0;
    double min = joint_tf[0].getOrigin().length(); 
    for(int i=1; i<joint_tf.size();++i){
        if(min >= joint_tf[i].getOrigin().length()){
            min = joint_tf[i].getOrigin().length();
            j = i;
        }else{
            min = min;
        }
    }

    return j;
}

void Collsion::collsion_avoidance(vector<tf::StampedTransform>& joint_tf, int id){

    // cout << this->goal_index << endl;
    // br.sendTransform(tf::StampedTransform(att_goals[goal_index], ros::Time::now(), "base", "attraction_goal"));
    // cout << att_goals.size() << endl;
    int obstacle_joint_index = judgeJoint(joint_tf);
    cout << joint_tf[0].getOrigin().length() << endl;
    cout << joint_tf[1].getOrigin().length() << endl;
    cout << joint_tf[2].getOrigin().length() << endl;
    cout << joint_tf[3].getOrigin().length() << endl;
    cout << joint_tf[4].getOrigin().length() << endl;
    cout << joint_tf[5].getOrigin().length() << endl;
    cout << "最近关节为： " << obstacle_joint_index << endl;
    cout << joint_tf[obstacle_joint_index].child_frame_id_ << endl;
    
    while(ros::ok()){
        try
        {
            tf_listen.waitForTransform("base", joint_tf[obstacle_joint_index].child_frame_id_, ros::Time(0), ros::Duration(3.0));
            tf_listen.lookupTransform("base", joint_tf[obstacle_joint_index].child_frame_id_, ros::Time(0), base_obstacle);
            tf_listen.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));
            tf_listen.lookupTransform("base", "tool0", ros::Time(0), base_tool);
            // tf_listen.waitForTransform("base", "attraction_goal", ros::Time(0), ros::Duration(3.0));
            // tf_listen.lookupTransform("base", "attraction_goal", ros::Time(0), base_tool);
        }
        catch(tf::TransformException ex)
        {
            continue;
        }

        tool_obstacle = joint_tf[obstacle_joint_index];
        break;
    }

    cout << tool_obstacle.getOrigin().length() << endl;
    //base_tool.setRotation(tf::Quaternion(0.977, 0.208, 0.011, 0.019));

    tf::Vector3 obstacle_tool = base_tool.getOrigin()-base_obstacle.getOrigin();
    tf::Vector3 distance = tool_obstacle.getOrigin();

    double tool0_obstacle_distance = distance.length();
    oposition = tf::Vector3(test.getOrigin().x(), test.getOrigin().y(), test.getOrigin().z());
    nopsition = tf::Vector3(tool_obstacle.getOrigin().x(), tool_obstacle.getOrigin().y(), tool_obstacle.getOrigin().z());
    n_o_distance = nopsition - oposition;

    double obstacle_speed = n_o_distance.length()*60;

    tf::Quaternion q_d(0.0,0.0,0.0,1.0);

    if(distance.length() > SECURITY_DISTANCE)
    {
        nh.setParam("avoid", false);
        repu_goal.setOrigin(tf::Vector3(base_tool.getOrigin().x(), base_tool.getOrigin().y(), base_tool.getOrigin().z()));
        repu_goal.setRotation(q_d);
        ostringstream oss;
        oss << "repulsiv_goal_" << id;
        br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", oss.str()));
    }
    else if(distance.length() < SECURITY_DISTANCE)
    {
        nh.setParam("avoid",true);
        if(obstacle_speed < SECURITY_SPEED)
        {
            float x = obstacle_tool.x()*1/((exp(weight*distance.length()))*distance.length()) + base_tool.getOrigin().x();
            float y = obstacle_tool.y()*1/((exp(weight*distance.length()))*distance.length()) + base_tool.getOrigin().y();
            float z = obstacle_tool.z()*1/((exp(weight*distance.length()))*distance.length()) + base_tool.getOrigin().z();
            repu_goal.setOrigin(tf::Vector3(x, y, z));
            repu_goal.setRotation(q_d);
            ostringstream oss;
            oss << "repulsiv_goal_" << id;
            br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", oss.str()));
        }
        else if(obstacle_speed > SECURITY_SPEED)
        {
            if(oposition.length() > nopsition.length()){
                float a = obstacle_tool.x()*1/((exp(weight*distance.length()))*distance.length());
                float b = obstacle_tool.y()*1/((exp(weight*distance.length()))*distance.length());
                float c = obstacle_tool.z()*1/((exp(weight*distance.length()))*distance.length());
                tf::Vector3 abc = tf::Vector3(a,b,c);
                float aa = -(b+c)/a;
                float bb = 1;
                float cc = 1;
                tf::Vector3 xyz = tf::Vector3(aa,bb,cc);
                float x = aa*abc.length()/xyz.length() + base_tool.getOrigin().x();
                float y = bb*abc.length()/xyz.length() + base_tool.getOrigin().y();
                float z = cc*abc.length()/xyz.length() + base_tool.getOrigin().z();
                repu_goal.setOrigin(tf::Vector3(x,y,z));
                repu_goal.setRotation(q_d);
                ostringstream oss;
                oss << "repulsiv_goal_" << id;
                br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", oss.str()));
            }
            else if(oposition.length() < nopsition.length()){
                repu_goal.setOrigin(tf::Vector3(base_tool.getOrigin().x(), base_tool.getOrigin().y(), base_tool.getOrigin().z()));
                repu_goal.setRotation(q_d);
                ostringstream oss;
                oss << "repulsiv_goal_" << id;
                br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", oss.str()));
            }
        }
    }

    while(ros::ok()){
        try{
            tf_listen.waitForTransform("tool0", "attraction_goal", ros::Time(0), ros::Duration(3.0));
            tf_listen.waitForTransform("tool0", "repulsiv_goal_" + to_string(id), ros::Time(0), ros::Duration(3.0));
            tf_listen.lookupTransform("tool0", "attraction_goal", ros::Time(0), attraction);
            tf_listen.lookupTransform("tool0", "repulsiv_goal_" + to_string(id), ros::Time(0), repulsive);
            break;
        }
        catch(tf::TransformException ex){
            ROS_ERROR(ex.what());
            continue;
        }
    }

    //bool check = false;
    //if(check){
        //goal.setOrigin(tf::Vector3(attraction.getOrigin().x() + repulsive.getOrigin().x() + repulsive_2.getOrigin().x(),attraction.getOrigin().y() + repulsive.getOrigin().y() + repulsive_2.getOrigin().y(),attraction.getOrigin().z() + repulsive.getOrigin().z() + repulsive_2.getOrigin().z()));
    //}else{
    double pen_coe_1 = 1.0;
    double pen_coe_2 = 1.5;
	goal.setOrigin(tf::Vector3(pen_coe_1 * attraction.getOrigin().x() + pen_coe_2 * repulsive.getOrigin().x(),
                               pen_coe_1 * attraction.getOrigin().y() + pen_coe_2 * repulsive.getOrigin().y(),
                               pen_coe_1 * attraction.getOrigin().z() + pen_coe_2 * repulsive.getOrigin().z()));

    // goal.setOrigin(tf::Vector3(attraction.getOrigin().x() + repulsive.getOrigin().x(),
    //                             attraction.getOrigin().y() +  repulsive.getOrigin().y(),
    //                            attraction.getOrigin().z() +  repulsive.getOrigin().z()));

	goal.setRotation(attraction.getRotation());
	br.sendTransform(tf::StampedTransform(goal, ros::Time::now(), "tool0", "desired_goal"));

	// if(fabs(base_tool.getOrigin().x()-att_goals[goal_index].getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -att_goals[goal_index].getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - att_goals[goal_index].getOrigin().z()) < 0.03)
	// {
    //     if(goal_index == 1 || goal_index == 4 || goal_index == 7 || goal_index == 10 || goal_index == 13){
    //         gripper.clamp();
    //         std::cout << goal_index << std::endl;
    //     }else if (goal_index == 0 || goal_index == 3 || goal_index == 6 || goal_index == 9 || goal_index == 12){
    //         gripper.loose();
    //         std::cout << goal_index << std::endl;
    //     }
        
    //     goal_index = target_queue.front();
    //     target_queue.pop();
    //     target_queue.push(goal_index);
	//}
}

