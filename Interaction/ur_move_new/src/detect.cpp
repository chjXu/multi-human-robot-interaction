#include<ur_move_new/detect.h>

void Object::init()
{
    cv::Mat cameraMatrix_origin = (Mat_<double>(3, 3) << 608.390625, 0.0, 311.2029724121094, 0.0, 607.375244140625, 239.69219970703125, 0.0, 0.0, 1.0); //realSense
    // cv::Mat cameraMatrix_origin = (Mat_<double>(3, 3) << 1061.45, 0.0, 959.19, 0.0, 1056.70, 578.16, 0.0, 0.0, 1.0);  //kinect
    //cv::Mat cameraMatrix_origin = (Mat_<double>(3, 3) << 610.18896484375, 0.0, 327.3924560546875, 0.0, 608.7219848632812, 250.08642578125, 0.0, 0.0, 1.0);  //usb_cam
    this->camera_matrix = cameraMatrix_origin.clone();
    cv::Mat distCoeffs_origin = (Mat_<double>(1,5) << 0,0,0,0,0);
    this->distCoeffs = distCoeffs_origin.clone();
    //监听camera 和 marker_0之间的tf信息
    //this->tf_listener();

    tf::Quaternion q_place(0.648,0.760,-0.0426,-0.0048);
    q_place.normalized();

    place_point.setOrigin(tf::Vector3(-0.630,-0.836,0.509));
    place_point.setRotation(q_place);

    tf::Quaternion q_init(0.0548034,0.998342,-0.00401,0.0171416);
    q_init.normalized();
    init_point.setOrigin(tf::Vector3(0.0,-0.388093,0.5));
    init_point.setRotation(q_init);
}

//读取图像
void Object::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // cout<<"--------------------------------"<<endl;
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr  =cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(this->img);
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s",ex.what());
        return;
    }
} 

void Object::locate_marker()
{
    //每执行完一次动作会重新进行图片的读取，以及物块marker的定位
    this->markerIds.clear();
    this->markerCorners.clear();
    this->tvecs.clear();
    this->rvecs.clear();
    this->object_list.clear();
    // this->idList.child_frame_list.clear();



    this->tf_br.sendTransform(tf::StampedTransform(place_point,ros::Time::now(),"base","place_point"));
    this->tf_br.sendTransform(tf::StampedTransform(init_point,ros::Time::now(),"base","init_point"));   

    if(img.empty()){
        cout<<"no image"<<endl;
    }

    if(!img.empty())
    {   
        // cv::resize(this->img, this->img, cv::Size(640,480));
        cv::aruco::detectMarkers(this->img,this->dictionary,this->markerCorners,this->markerIds);  //检测marker


        if(markerIds.size() > 0){
            // cout<<"markerIds:  "<<endl;
            // for(int i = 0;i<markerIds.size();i++)
            // {
            //     cout<<markerIds[i]<<"  ";
            // }
            // cout<<endl;

            cv::aruco::drawDetectedMarkers(this->img,this->markerCorners,this->markerIds);
            for(int idx = 0;idx < this->markerIds.size(); idx++)   //this->markerIds.size()
            {
                int marker_id =  markerIds[idx];
                if((marker_id > 0) && (marker_id <= 10))  //这里不用发送marker_0的tf信息了
                {
                    // cout<<markerIds[idx]<<endl;
                    cv::aruco::estimatePoseSingleMarkers(this->markerCorners,0.05,this->camera_matrix,this->distCoeffs,this->rvecs,this->tvecs);
                    cv::aruco::drawAxis(this->img,this->camera_matrix,this->distCoeffs,this->rvecs[idx],this->tvecs[idx],0.1); 
                    this->sendMarkerTf(this->tvecs[idx],this->rvecs[idx],marker_id);
                    //object_list.push_back(o_temp);
                }
            }

            // auto target_marker = std::min_element(object_list.begin(), object_list.end(), Object::comp);
            // cout << target_marker->id << endl;

            // //send
            // for(int i=0; i<target_marker->object_transformation.size(); ++i){
            //     attraction_goal = target_marker->object_transformation[0];
            //     tf_br.sendTransform(tf::StampedTransform(attraction_goal, ros::Time::now(), "base","attration_goal"));
                
            //     // if(isArravied()){
            //     //     ++i;
            //     // }
            //     // else{
            //     //     i = i;
            //     // }
            // }
        }

        

        // cv::resize(img,img,cv::Size(960,540));
        //cv::imshow("detect",img);
        // cv::imwrite("./marker_5.jpg", markerImage);
        //cv::waitKey(3);
    }
}

bool Object::isArravied(){
    while(ros::ok())
    {
        try
        {
            tf_listener.waitForTransform("base","tool0",ros::Time(0),ros::Duration(3.0));
            tf_listener.lookupTransform("base","tool0",ros::Time(0),base_tool);
            tf_listener.waitForTransform("base","attration_goal",ros::Time(0),ros::Duration(3.0));
            tf_listener.lookupTransform("base","attration_goal",ros::Time(0),base_attraction);

            break;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            continue;
        }
    }


    if(fabs(base_tool.getOrigin().x()-base_attraction.getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -base_attraction.getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - base_attraction.getOrigin().z()) < 0.03)
    {
        return true;
    }else{
        return false;
    }

}

tf::Quaternion Object::getQuaternion(cv::Mat &rotation_matrix)
{
    double trace = rotation_matrix.at<double>(0,0) + rotation_matrix.at<double>(1,1) + rotation_matrix.at<double>(2,2);
    double W = sqrt(trace + 1) / 2.0;
    double X = (rotation_matrix.at<double>(1,2) - rotation_matrix.at<double>(2,1)) / (4.0 * W);
    double Y = (rotation_matrix.at<double>(2,0) - rotation_matrix.at<double>(0,2)) / (4.0 * W);
    double Z = (rotation_matrix.at<double>(0,1) - rotation_matrix.at<double>(1,0)) / (4.0 * W);
    tf::Quaternion q(X,Y,Z,W);
    return q;
}


void Object::sendMarkerTf(cv::Vec3d &marker_trans,cv::Vec3d& marker_rot,int &id)  //marker_rot--旋转向量
{

	cv::Mat rot(3, 3, CV_64FC1);
	static tf::TransformBroadcaster marker_position_broadcaster;
    cv::Rodrigues(marker_rot, rot);   //罗得里格斯公式 旋转矩阵  相机 --> 物块marker
    rot.convertTo(rot, CV_64FC1);

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                        rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                        rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    // tf::Quaternion q = this->getQuaternion(rot);  //旋转矩阵 --> 四元数

    tf::Vector3 tf_trans(marker_trans(0), marker_trans(1), marker_trans(2));  //cv -> tf

    tf::Transform transform(tf_rot,tf_trans);    //这里得到的是相机到物块的marker的位姿
    tf::Transform transform_new , transform_target_new;

    //矫正点
    auto q_1 = transform.getRotation();
    q_1.setRPY(0,M_PI,0);  //以当前的marker的姿态进行绕轴旋转
    transform_new.setRotation(q_1);
    transform_new.setOrigin(tf::Vector3(0 + 0.0,0 + 0.0, 0 + 0.18));   //矫正点

    //目标点
    auto q_2 = transform.getRotation();
    q_2.setRPY(0,M_PI,0);
    transform_target_new.setRotation(q_2);
    transform_target_new.setOrigin(tf::Vector3(0 + 0.0,0 + 0.0, 0 + 0.18));  //目标点，也是抓取点
    // transform = transform.inverse();　　//这里发布的tf信息不用进行反转


    ostringstream oss_ori,target,correct;
    oss_ori << "marker_" << id;
    target << "target_" << id;
    correct << "correct_" <<id;

    marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Realsense",oss_ori.str()));
    marker_position_broadcaster.sendTransform(tf::StampedTransform(transform_target_new, ros::Time::now(), oss_ori.str(),target.str()));
    marker_position_broadcaster.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(),oss_ori.str(),correct.str()));

}


void *updateRealsenseCamera(void *ptr){
    tf::TransformBroadcaster br;
    tf::Transform rect_goal, camera_transform;
    tf::TransformListener tf_listener;
    tf::StampedTransform realsense_goal;
    
    ros::Rate rate(60);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base","tool0",ros::Time(0),realsense_goal);
        }
        catch(tf::TransformException &ex){
            //ROS_INFO("camera thread...");
            continue;
        }
        
        //获得之后
        auto q = realsense_goal.getRotation();
        q.setRPY(0.0,0.0,0.0);
        camera_transform.setRotation(q);
        camera_transform.setOrigin(tf::Vector3(-0.07,-0.04,0));
        br.sendTransform(tf::StampedTransform(camera_transform,ros::Time::now(),"tool0","Realsense"));

        ros::spinOnce();
    }
}

void Object::run_detect(){
    ros::Rate rate(30);
    // ros::spinOnce();
    
    pthread_create(&this->updateRealsense, NULL, updateRealsenseCamera, NULL);

    while(ros::ok()){ 
        //ros::spinOnce();//读取图片
        this->locate_marker();//定位marker并发送marker的信息
        // this->id_pub.publish(this->idList);　//这个本来是想发送marker的id信息的。
        // this->tf_br.sendTransform(tf::StampedTransform(place_point,ros::Time::now(),"base","place_point"));
        // this->tf_br.sendTransform(tf::StampedTransform(init_point,ros::Time::now(),"base","init_point"));   
        ros::spinOnce();
    }
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"object_dected");
	Object realSense(1,"/camera");
    realSense.run_detect();

    return 0;
}
