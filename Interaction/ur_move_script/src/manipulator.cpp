#include <ur_move_script/manipulator.h>


// int count = 0;

void Manipulator::init(){
    //关节名称
	joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

    this->mode = mode1;
    this->toPlace = true;
    this->update = true;
}

string Manipulator::getModeState()
{
    string current_mode = this->mode;
    return current_mode;
}

bool Manipulator::isToPlace()
{
    bool is_toPlace = this->toPlace;
    return is_toPlace;
}


void* Manipulator::goal_rectify__thread(void *arg)
{
    tf::TransformBroadcaster br;
    tf::Transform rect_goal;
    tf::TransformListener tf_listener;
    tf::StampedTransform desired_goal;

    double work_space_radius = 0.85;   //工作空间
    double base_radius = 0.2;  //

    ros::Rate rate(60);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;  //对姿态进行修正

            rect_goal.setRotation(desired_goal.getRotation());
            rect_goal.setOrigin(desired_goal.getOrigin());
            if (goal_position.z() < 0.15){
                rectified_position.setZ(0.15);
                rect_goal.setOrigin(rectified_position);
                ROS_WARN("goal under the working surface, setting rectified goal\n");
            }

            double desired_distance = rectified_position.length();

            if (desired_distance > work_space_radius)
            {
                double scale = work_space_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal outoff robot work space! setting rectified goal\n");
            }

            desired_distance = rectified_position.length();
            if (desired_distance < base_radius){
                double scale = base_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal too close to base! setting rectified goal\n");
            }
            br.sendTransform(tf::StampedTransform(rect_goal, ros::Time::now(), "base", "rect_goal"));
        }
        catch(tf::TransformException ex){
        }
        rate.sleep();
    }
}

void Manipulator::jointStateCallback(const sensor_msgs::JointState& joint_state){
    joint_angles.clear();  //关节角度
    joint_speed.clear();   //关节速度
    std::vector<std::string> joint_names_recv = joint_state.name;  //关节名称  .name是一个列表
    for(auto it = joint_names.begin(); it !=joint_names.end(); ++it)
    {
        for(auto it_recv = joint_names_recv.begin(); it_recv != joint_names_recv.end(); ++it_recv)
        {
            if (*it_recv == *it)
            {
                int idx = it_recv - joint_names_recv.begin();
                int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);  //关节角度 ，其微分就是角速度 ， 即下面的joint_speed
                joint_speed.push_back(joint_state.velocity[idx]);   //关节角速度
                break;
            }
        }
    }
    JointState_available = true;
}

// void Manipulator::idCallback(const object_msgs::childFrameList& id_list){
//     for(int i = 0;i<id_list.child_frame_list.size();i++){
//         object_msgs::childFrameId id = id_list.child_frame_list[i];
//         this->id_vec.push_back(id);
//     }
// }

void Manipulator::listener_object(){
    int id = 1;
    string target = "target_" + to_string(id);
    string correct = "correct_" + to_string(id);
    string object = "marker_" + to_string(id);
    while(ros::ok())
    {   
        // int id = 1;
        // string target = "target_" + to_string(id);
        // string correct = "correct_" + to_string(id);

        this->correct_target.clear();
        try{
            // cout<<"监听的id是："<<id<<endl;
            //矫正点　－－　目标点
            this->tf_listener.lookupTransform("base", correct , ros::Time(0), transform_correct);  //监听你发送的tf信息
            correct_target.push_back(transform_correct);   //问题：如果到达目标点之后，被遮挡了，就会一直在这里监听
            this->tf_listener.lookupTransform("base", target ,ros::Time(0),transform_target);
            correct_target.push_back(transform_target);

            //为了判断动没有动，监听marker的信息
            this->tf_listener.lookupTransform("base",object,ros::Time(0),marker_current);
            if(this->update){
                this->correct_target_origin.clear();
                this->correct_target_origin.push_back(transform_correct);
                this->correct_target_origin.push_back(transform_target);
                this->marker_origin = marker_current;
                this->update = false;
            }
            //-----------------------------

            tf_available = true;
            break;
        }
        catch(tf::TransformException &ex){
            id += 1;
            if(id >= 25)
                id = 1;
            target = "target_" + to_string(id);
            correct = "correct_" + to_string(id);
            object = "marker_" + to_string(id); 

            this->tf_available = false;
            continue;
        } 
        std::cout << JointState_available<< std::endl;

    }
}

void Manipulator::listener_point(){
    while(ros::ok()){
        this->place_init.clear();
        try{
            this->tf_listener.lookupTransform("base","place_point",ros::Time(0),place_point);
            place_init.push_back(place_point);
            this->tf_listener.lookupTransform("base","init_point",ros::Time(0),init_point);
            place_init.push_back(init_point);
            break;           
        }
        catch(tf::TransformException &ex){
            continue;
        }
    }
}

void Manipulator::sendCmd(TRAC_IK::TRAC_IK &ik_solver , int &idx)
{
    if(this->mode == mode1){
        cout<<"correct_target size : "<<correct_target_origin.size()<<endl;
        transformTFToKDL(correct_target_origin[idx], desired_pose);   //transformTFToKDL：运动学/动力学库，将监听得到的tf信息转换到KDL中
        cout<<"current child_id : "<<correct_target_origin[idx].child_frame_id_<<endl;
    }
    else if(this->mode == mode2){
        cout<<"place_init size : "<<place_init.size()<<endl;
        transformTFToKDL(place_init[idx], desired_pose);   //transformTFToKDL：运动学/动力学库，将监听得到的tf信息转换到KDL中
        cout<<"current _id : "<<place_init[idx].child_frame_id_<<endl;        
    }
     
    //关节状态获取得到   tf获取得到
    if(JointState_available && tf_available){

        for(int i = 0; i < 6; ++i)
            {
                q(i) = joint_angles[i]; //q是joint_angles（相当于上一时刻）
                printf("q_current[%d]:%f\n",i,joint_angles[i]);
            }
        //int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, 
        //                             KDL::JntArray& return_joints, KDL::Twist tolerances);
        if(ik_solver.CartToJnt(q, desired_pose, q_desired))    //返回期望的q_desired，逆运动学求解器
        {
            std::vector<double> cmd_vector;

            //  sensor_msgs::JointState joint_publish;
            // joint_publish.header.stamp = ros::Time::now();
            // joint_publish.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

            for(int i = 0; i < 6; ++i)
            {

                //joint_publish.position.push_back(q_desired(i));
                //   printf("q_desired[%d]:%f\n",i,q_desired(i));
                double delta = q_desired(i) - q(i);  //误差（q_desired:系统输入，q：上一时刻的“输出”）
                double speed = Kp*delta + Td*joint_speed[i]; //PD控制--比例环节:当前偏差，微分环节：最近偏差

                //速度判断
                if(speed > max_speed){
                    speed = max_speed;
                }
                if(speed < -max_speed){
                    speed = -max_speed;
                }

                cmd_vector.push_back(speed);
                printf("speed[%d]:%f\n",i,speed);   //速度控制
            }

            // ur_joint_publisher.publish(joint_publish);

            char cmd[100];  //command
            sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)", cmd_vector[0], cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
            ROS_INFO("ur script send: %s", cmd);

            cmd_msg.data = cmd;
            ur_cmd_publisher.publish(cmd_msg);
        }
    }

}

//判断标准：marker和机械臂末端的距离小于1cm
bool Manipulator::judge(int &idx)
{
    double length = 0;
    double q_delta = 0;
    tf::Vector3 desired_vec;
    tf::Vector3 current_vec;
    tf::Quaternion desired_q;
    tf::Quaternion current_q;

    //mode1:"track_obj" ; mode2:"track_point"
    if(this->mode == mode1){
        desired_vec = correct_target_origin[idx].getOrigin();//这个原来是：correct_target
        // desired_q = correct_target_origin[idx].getRotation();  //四元数
    }
    else if(this->mode == mode2){
        desired_vec = place_init[idx].getOrigin();
        // desired_q = place_init[idx].getRotation();   //四元数
    }

    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base","tool0",ros::Time(0),transform_current);
            current_vec = transform_current.getOrigin();
            current_q = transform_current.getRotation();  //四元数
            break;
        }
        catch(tf::TransformException ex){
            continue;
        }
    }


    tf::Vector3 vec = desired_vec - current_vec;
    length = vec.length();
    // tf::Quaternion q_ = desired_q - current_q;
    // q_delta = q_.length();
    // cout<<"q_.length() == "<<q_delta<<endl;   //这个判定条件不要了

    if( idx == 0 && length <= 0.01)
        return true;
    else if(idx == 1 && length <= 0.01)
        return true;

    return false;
}

bool Manipulator::judge_update(){
    double length = 0;
    tf::Vector3 vec_current = this->marker_current.getOrigin();
    tf::Vector3 vec_origin = this->marker_origin.getOrigin();
    tf::Vector3 vec = vec_current - vec_origin;
    length = vec.length();
    if(length>=0.1){
        this->correct_target_origin.clear();
        for(int i = 0; i < this->correct_target.size();i++){
            this->correct_target_origin.push_back(correct_target[i]);
        }
        this->marker_origin = marker_current;
        count++;
        return true;
    }
    
    return false;
}

void Manipulator::track()  
{
    //pthread_create(&rectify_thread, NULL, goal_rectify__thread, NULL);
    ros::spinOnce();
    ros::Rate rate(30);
    std::cout << "controlling using time step at: " << TIME_STEP << std::endl;
    TRAC_IK::TRAC_IK ik_solver(base,tip,urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    if(this->mode == mode1){
        int idx = 0;
        while(ros::ok()){
            
            this->listener_object();

            //监听完之后，如果当前的marker坐标移动了，那么就令idx = 0,重新到新的矫正点
            if(this->judge_update()){
                idx = 0;
            }

            this->sendCmd(ik_solver,idx);

            if(idx == 0 && this->judge(idx))
            {
                cout<<"current idx : "<<idx<<endl;
                cout<<"-------------------------------------------------------"<<endl;
                sleep(1);
                idx++;
            }
                
            if(idx == 1 && this->judge(idx))
            {
                // sleep(0.5);
                // // 再监听一次
                // this->listener_object();
                // if(!this->judge(idx)){     //中途移动的话，就重新循环
                //     idx == 0;
                //     continue;
                // }
                
                this->mode = mode2;
                this->update = true;
                cout<<"current idx : "<<idx<<endl;
                cout<<"------------跳出mode1循环------------------------------"<<endl;
                break;   //达到了目标点，跳出　--> 抓取
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    else if(this->mode == mode2){
        int idx = 0;
        if(this->toPlace){
            this->toPlace = false;
            // cout<<"toPlace : "<<this->toPlace<<endl;
            // sleep(3);
        }
        else{
            idx = 1;
            this->toPlace = true;
            // this->mode = mode1;   //这个标志位错了
        }
        while(ros::ok())
        {
            cout<<"---------------mode2循环中-------------------"<<endl;
            this->listener_point();
            this->sendCmd(ik_solver,idx);

            //0:到达放置点 ; 1:到达init点
            if(idx == 0 && this->judge(idx)){
                break;
            }
            else if(idx == 1 && this->judge(idx)){
                this->mode = mode1;
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
}
