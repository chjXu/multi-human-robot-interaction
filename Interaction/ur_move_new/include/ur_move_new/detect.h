#pragma once
#include<iostream>
#include<vector>
#include<sstream>
#include<algorithm>
#include<string>
// #include<map>
//-----------opencv&aruco---------//
//#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/aruco.hpp>
#include<opencv2/aruco/dictionary.hpp>
//-----------ros------------//
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>

//----------toCv------------//
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>

#include <pthread.h>
//----------marker 信息 -----//
// #include <object_msgs/childFrameId.h>
// #include <object_msgs/childFrameList.h>
// #include <object_msgs/singleChildFrame.h>

using namespace std;
using namespace cv;

struct Object_list{
    vector<tf::Transform> object_transformation;
    int id;
};

//类：物块
class Object
{
public:
    Object(int camera_index , string camera_type):it(nh),_camera_index(camera_index)
    {
        char camera_topic[50];
        string img_topic = camera_type + "/color/image_raw";   //realSense
        //string topic = "/camera_1/hd/image_color_rect";  //cv_camera
        // string img_topic = "/usb_cam/image_raw";  //usb_cam
        // camera_type += img_topic;

        this->init();
        // id_pub = nh.advertise<object_msgs::childFrameList>("child_frame_id_list",10);

        //订阅图像话题
        image_sub = it.subscribe(img_topic,1,&Object::img_callback,this);
    }

    Object():it(nh){
        // cout<<"无参构造"<<endl;
        // string img_topic = "/color/image_raw";   //realSense
        string img_topic = "/camera_1/hd/image_color_rect";  //cv_camera
        this->init();
        // id_pub = nh.advertise<object_msgs::childFrameList>("child_frame_id_list",10);
        image_sub = it.subscribe(img_topic,1,&Object::img_callback,this);
    }

    static bool comp(Object_list& obj_1, Object_list& obj_2){
        return obj_1.id < obj_2.id;
    }

    void init();
    void run_detect();
    void img_callback(const sensor_msgs::ImageConstPtr& msg); 
    void locate_marker();
    void send_desired_goal();
    void sendMarkerTf(cv::Vec3d& marker_trans,cv::Vec3d& marker_rot,int &id);
    void sendCameraTf();
    tf::Quaternion getQuaternion(cv::Mat &rotation_matrix);
    bool isArravied();


private:
    //相机
    int _camera_index;
    string _camera_type;

    //相机内参
    cv::Mat camera_matrix;
    cv::Mat distCoeffs;
    //相机外参
    string base_ = "marker_0";
    string child_ = "Realsense";
    // Eigen::Matrix3d rot;
    // Eigen::Matrix<double,3,1> trans;
    // Eigen::Matrix4d T , T_new;

    ros::NodeHandle nh;
    ros::Publisher id_pub;
    image_transport::ImageTransport it;  //读取图片的句柄
    image_transport::Subscriber image_sub;
    // message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor;
    cv::Mat img;

    //相机的tf信息
    tf::StampedTransform base_tool, base_attraction;
    tf::StampedTransform end_stamped , camera_stamped;
    tf::Transform camera_transform;   //要发送的相机的tf信息

    //marker
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    vector<int> markerIds;
    vector<vector<cv::Point2f> > markerCorners;
    vector<cv::Vec3d> rvecs;
    vector<cv::Vec3d> tvecs;

    tf::TransformBroadcaster tf_br;
    tf::TransformListener tf_listener;

    tf::Transform place_point;   //放置点
    tf::Transform init_point;    //初始点

    pthread_t updateRealsense;

    tf::Transform attraction_goal;
    vector<Object_list> object_list;

    // object_msgs::childFrameList idList;
    // object_msgs::childFrameId frame_id;
    // object_msgs::singleChildFrame singleFrame;

    //用于存放所有的tf::StampedTransform信息
    // map<int , int> relation;
    // vector<tf::StampedTransform> stamped_obj_vec;
};