#include <human_pose/visualizate.h>

using namespace HumanPose;
using namespace std;

const string action_classes[5] = {"T-Pose", "stand", "sit", "operate","operate"};


void Visualizate::addCameraInfo(Eigen::Matrix3d& rot, Eigen::Matrix<double, 3, 1>& trans, cv::Mat& cameraMatrix){
    this->rot = rot;
    this->trans = trans;
    this->cameraMatrix = cameraMatrix;
}

map<int, vector<Pose_3d>> Visualizate::show2DPose(vector<HumanPose::Pose_Info>& person_info, vector<vector<int>> &new_index_id, int frame_index){
    if(person_info.empty() || new_index_id.empty()){
        return {};
    }

    std::map<int, vector<Pose_3d>> results;
    // for(int i=0; i<new_index_id.size(); ++i){
    //     for(int j=0; j<new_index_id[i].size(); ++j){
    //         cout << new_index_id[i][j] << " ";
    //     }
    // }


    for(int i=0; i<new_index_id.size(); ++i){
        int label = new_index_id[i][1];
        vector<HumanPose::Pose_3d> pose_3d = person_info[new_index_id[i][0]].pose_3d;

        int action = person_info[new_index_id[i][0]].action;
        //smooth
        // pose_3d = smooth(pose_3d);        
        
        results.insert(std::pair<int, vector<Pose_3d>>(label, pose_3d));

        // 投影
        vector<cv::Point2d> pose_2d;
        for(int j=0; j<pose_3d.size(); ++j){
            cv::Point2d joint_2d = project_from_world(pose_3d[j]);
            pose_2d.push_back(joint_2d);
        }

        for(int p=0; p<pose_2d.size(); ++p){
            cv::circle(image, cv::Point(pose_2d[p].x, pose_2d[p].y), 4, this->body_color[label], 5);
        }

        string action_result = action_classes[action];
        string text;
        if(label == 0 || label == 1){
            text = "Operator";
            //sprintf(text, "Operator");
        }else{
            text = "Non-Operator";
            //sprintf(text, "Non-Operator");
        }
        
        cv::putText(image, "id: " + to_string(label), cv::Point(pose_2d[1].x - 80, pose_2d[1].y - 150), cv::FONT_HERSHEY_SIMPLEX, 1.0, this->body_color[label], 2);
        cv::putText(image, "it: " + text, cv::Point(pose_2d[1].x - 80, pose_2d[1].y - 120), cv::FONT_HERSHEY_SIMPLEX, 1.0, this->body_color[label], 2);
        cv::putText(image, "ac: " + action_result, cv::Point(pose_2d[1].x - 80, pose_2d[1].y - 90), cv::FONT_HERSHEY_SIMPLEX, 1.0, this->body_color[label], 2);
        // cv::putText(image, "Frame:" + to_string(frame_index), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("Camera", image);
    cv::waitKey(3);

    if(!results.empty()){
        return results;
    }else{
        return {};
    }
    

    // while(ros::ok()){
    //     char c = cv::waitKey(30);
    //     if(c == ' '){
    //         break;
    //     }
    // }
}

void Visualizate::show2DPose(vector<HumanPose::Pose_Info>& person_info, int frame_index, int fps){
    if(person_info.empty()){
        return;
    }

    for(int i=0; i<person_info.size(); ++i){
        vector<HumanPose::Pose_3d> pose_3d = person_info[i].pose_3d;

        // 投影
        vector<cv::Point2d> pose_2d;
        for(int j=0; j<pose_3d.size(); ++j){
            cv::Point2d joint_2d = project_from_world(pose_3d[j]);
            pose_2d.push_back(joint_2d);
        }

        for(int p=0; p<pose_2d.size(); ++p){
            cv::circle(image, cv::Point(pose_2d[p].x, pose_2d[p].y), 4, this->body_color[i], 5);
            //cv::line(image, cv::Point(pose_2d[0].x, pose_2d[0].y), cv::Point(pose_2d[1].x, pose_2d[1].y), cv::Scalar(0,0,255),2);
        }

        char text[10];
        sprintf(text, "id:%d", person_info[i].label);
        cv::putText(image, text, cv::Point(pose_2d[1].x - 40, pose_2d[1].y - 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, this->body_color[i], 2);
        cv::putText(image, "Frame:" + to_string(frame_index), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1);
        //cv::putText(image, "FPS:" + to_string(fps), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1);
        //cv::resize(image, image, cv::Size(960, 540));
    }

    cv::imshow("Camera", image);
    //cv::waitKey(3);

    while(ros::ok()){
        char c = cv::waitKey(30);
        if(c == ' '){
            break;
        }
    }
}

void Visualizate::Publish3DPoses(vector<HumanPose::Pose_Info>& pose_3d){
    if(pose_3d.empty()){
        return;
    }

    for(int i=0; i<pose_3d.size(); ++i){
        visualization_msgs::Marker line_list;
        line_list.id = i;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        line_list.header.frame_id = "marker_0";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "humans";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;

        line_list.scale.x = 0.01;
        line_list.scale.y = 0.01;
        line_list.scale.z = 0.01;
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        for(int j=0; j<body_edge.size()/2; ++j){
            addLine(line_list, pose_3d[i].pose_3d, body_edge[2*j], body_edge[2*j+1]);
        }
        pose_pub->publish(line_list);
    }
}

void Visualizate::addLine(visualization_msgs::Marker& line_list, vector<HumanPose::Pose_3d>& pose, int a, int b){
    if(pose[a].available && pose[b].available){
        geometry_msgs::Point p_a, p_b;
        p_a.x = pose[a].x;
        p_a.y = pose[a].y;
        p_a.z = pose[a].z;

        p_b.x = pose[b].x;
        p_b.y = pose[b].y;
        p_b.z = pose[b].z;

        line_list.points.push_back(p_a);
        line_list.points.push_back(p_b);
    }
}


cv::Point2d Visualizate::project_from_world(const Pose_3d& pose_3d)
{
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    if(!pose_3d.available) {
        return cv::Point2d(0.0, 0.0);
    }

    Eigen::Matrix<double,3,1> World_point;
    Eigen::Matrix<double,3,1> Cam_point;
    World_point(0,0) = pose_3d.x;
    World_point(1,0) = pose_3d.y;
    World_point(2,0) = pose_3d.z;

    Cam_point = this->rot.transpose() * (World_point - this->trans);

    cv::Point2d image_cord;
    image_cord.x = (int)(Cam_point(0,0) * fx / Cam_point(2,0) + cx);
    image_cord.y = (int)(Cam_point(1,0) * fy / Cam_point(2,0) + cy);

    return image_cord;
}

void Visualizate::Publish3DPosesTF(std::map<int, vector<Pose_3d>> results){
    if(results.empty()){
        return;
    }

    std::map<int, vector<Pose_3d>> results_smooth;

    // if(!pre_results.empty()){
    //     for(auto iter:pre_results){
    //         cout << iter.first << endl; //id
    //         cout << iter.second << endl; //pose

    //         auto cur_pose = results.find(iter.first);
    //         vector<Pose_3d> s_pose = smooth(iter.second, cur_pose);
    //         results_smooth.insert(std::pair<int, vector<Pose_3d>>(iter.first, s_pose));
    //     }
    // }
    // pre_results = results;

    TFPose.clear();
    //ROS_INFO("results size: %d", results.size());
    for(auto iter=results.begin(); iter!=results.end(); ++iter){
        PoseTF pose_tf;
        pose_tf.person_id = iter->first;
        try
        {
            pose_tf.joint[0].setOrigin(tf::Vector3(iter->second[0].x, iter->second[0].y, iter->second[0].z));
            pose_tf.joint[1].setOrigin(tf::Vector3(iter->second[5].x, iter->second[5].y, iter->second[5].z));
            pose_tf.joint[2].setOrigin(tf::Vector3(iter->second[4].x, iter->second[4].y, iter->second[4].z));
            pose_tf.joint[3].setOrigin(tf::Vector3(iter->second[3].x, iter->second[3].y, iter->second[3].z));
            pose_tf.joint[4].setOrigin(tf::Vector3(iter->second[11].x, iter->second[11].y, iter->second[11].z));
            pose_tf.joint[5].setOrigin(tf::Vector3(iter->second[10].x, iter->second[10].y, iter->second[10].z));
            pose_tf.joint[6].setOrigin(tf::Vector3(iter->second[9].x, iter->second[9].y, iter->second[9].z));


            pose_tf.joint[0].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            pose_tf.joint[1].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            pose_tf.joint[2].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            pose_tf.joint[3].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            pose_tf.joint[4].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            pose_tf.joint[5].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            pose_tf.joint[6].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        }
        catch(const std::exception& e)
        {
            continue;
            std::cerr << e.what() << '\n';
        }
        TFPose.push_back(pose_tf);
    }

    for(int i=0; i<TFPose.size(); ++i){
        for(int j=0; j<TFPose[i].joint.size(); ++j){
            ostringstream oss;
            oss << "human_" << TFPose[i].person_id << joint_name[j];
            br->sendTransform(tf::StampedTransform(TFPose[i].joint[j], ros::Time::now(), "marker_0", oss.str()));
        }
    }

}

vector<HumanPose::Pose_3d> Visualizate::smooth(vector<HumanPose::Pose_3d>& pose_3d){
    // queue_results.push(resluts);
    // if(queue_results.size() == 2){
        
    // }
    return {};
}

void Visualizate::averagePose(Pose_3d& current_pose, Pose_3d& previous_pose, double alpha){
    Pose_3d ave_pose;
    if(alpha < 0 || alpha > 1){
        return ;
    }
    ave_pose.x = current_pose.x * alpha + (1.0 - alpha) * previous_pose.x;
    ave_pose.y = current_pose.y * alpha + (1.0 - alpha) * previous_pose.y;
    ave_pose.z = current_pose.z * alpha + (1.0 - alpha) * previous_pose.z;
}

void Visualizate::recognizating(vector<HumanPose::Pose_Info>& person_info){
    // for(int p=0; p<init_pose.size(); ++p){
    //     cv::circle(image, cv::Point(init_pose[p][0], init_pose[p][1]), 4, this->body_color[4], 2);
    // }
    // for(int e=0; e<body_edge.size()/2; ++e){
    //     cv::line(image, cv::Point(init_pose[body_edge[2*e]][0], init_pose[body_edge[2*e]][1]), cv::Point(init_pose[body_edge[2*e+1]][0], init_pose[body_edge[2*e+1]][1]), this->body_color[4],5);
    // }

    for(int i=0; i<person_info.size(); ++i){
        vector<HumanPose::Pose_3d> pose_3d = person_info[i].pose_3d;
        int action = person_info[i].action;
        // 投影
        vector<cv::Point2d> pose_2d;
        for(int j=0; j<pose_3d.size(); ++j){
            cv::Point2d joint_2d = project_from_world(pose_3d[j]);
            pose_2d.push_back(joint_2d);
        }

        for(int p=0; p<pose_2d.size(); ++p){
            cv::circle(image, cv::Point(pose_2d[p].x, pose_2d[p].y), 4, this->body_color[i], 5);
        }
        for(int e=0; e<body_edge.size()/2; ++e){
            cv::line(image, cv::Point(pose_2d[body_edge[2*e]].x, pose_2d[body_edge[2*e]].y), 
                            cv::Point(pose_2d[body_edge[2*e+1]].x, pose_2d[body_edge[2*e+1]].y),
                            this->body_color[i],5);
        }

        string action_result = action_classes[action];

        cv::putText(image, action_result, cv::Point(pose_2d[1].x - 40, pose_2d[1].y - 70), cv::FONT_HERSHEY_SIMPLEX, 1.5, this->body_color[i], 6);
    }
    
    if(!this->image.empty()){
        cv::imshow("Camera", this->image);
        cv::waitKey(3);
    }
}
