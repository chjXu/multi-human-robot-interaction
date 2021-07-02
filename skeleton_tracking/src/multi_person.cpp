#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <human_pose/pose.h>
#include <human_pose/visualizate.h>
#include <human_pose/track.h>
#include <chrono>

using namespace std;
using namespace cv;
using namespace chrono;
using namespace HumanPose;


class CameraType{
public:
    CameraType(){}
    virtual void loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
    virtual ~CameraType(){}

    virtual cv::Mat& getCameraIntrinsic() = 0;

private:
};

class KinectV2 : public CameraType{
public:
    KinectV2(): scale(1.0){}

    virtual void loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

    virtual cv::Mat& getCameraIntrinsic() const{
        //return this->cameraMatrix;
    }

private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    const double scale;
};

class USBCam : public CameraType{
public:
    USBCam():scale(1.0){}

    virtual void loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    const double scale;
};

/************************************************************/

class ImageProcess{
public:
    vector<HumanPose::Pose_Info> person_info;
    cv::Mat cameraMatrix;//内参
    //外参
    Eigen::Matrix3d rot;
    Eigen::Matrix<double, 3, 1> trans;
    
    cv::Mat image;

public:
    ImageProcess(int camera_index, string camera_type, string calib_path):it(nh), 
        _camera_index(camera_index), _calib_path(calib_path), scale(1.0), pose_check(false)
    {
        //本代码首先使用图片代替
        string image_topic = "/hd/image_color";
        image_sub = it.subscribe(camera_type+image_topic, 1, &ImageProcess::imageCallback, this);

        human_keypoint_sub = nh.subscribe("/HumanPose_3d_" + to_string(_camera_index), 1, &ImageProcess::human_keypoints_callback, this);

        initCalibration(scale);

        tf_listener = new tf::TransformListener;
    }

    ~ImageProcess(){
        // delete tf_listener;
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void initCalibration(double scale);
    void loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, double& scale);

    void human_keypoints_callback(const human_pose_msgs::HumanList humanList);

    //手动赋予相机外参
    void waitForCameraPose();

    vector<double> getCameraParam();

private:
    int _camera_index;
    string _calib_path;
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    //相机内参
    cv::Mat distCoeffs;
    const double scale;

    ros::Subscriber human_keypoint_sub;
    tf::TransformListener *tf_listener;

    bool pose_check;

};

inline
void ImageProcess::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat color_mat = cv_bridge::toCvShare(msg, "bgr8")->image;
        image = color_mat.clone();
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }
    
    // cv::resize(image, image, cv::Size(960, 540));
    // cv::imshow("camera_" + to_string(_camera_index), image);
    // cv::waitKey(3);
}

inline
void ImageProcess::loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, double& scale)
{
    cv::FileStorage fs;
    cv::Mat cameraMatrix_origin;

    string calib_path_ = _calib_path + "/calib_color.yaml";
    cout << calib_path_ << endl;
    if(fs.open(calib_path_, cv::FileStorage::READ)){
        fs["cameraMatrix"] >> cameraMatrix_origin;
        cout << "Matrix load success" << endl;
        cameraMatrix = cameraMatrix_origin.clone();
        cameraMatrix.at<double>(0, 0) *= scale;
        cameraMatrix.at<double>(1, 1) *= scale;
        cameraMatrix.at<double>(0, 2) *= scale;
        cameraMatrix.at<double>(1, 2) *= scale;

        distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
        fs.release();
    }
}

inline 
void ImageProcess::initCalibration(double scale)
{
    loadCalibrationFiles(_calib_path, cameraMatrix, distCoeffs, scale);
}

vector<double> ImageProcess::getCameraParam(){
    vector<double> params(4);
    params[0] = this->cameraMatrix.at<double>(0, 0);
    params[1] = this->cameraMatrix.at<double>(1, 1);
    params[2] = this->cameraMatrix.at<double>(0, 2);
    params[3] = this->cameraMatrix.at<double>(1, 2);

    return params;
}

void ImageProcess::human_keypoints_callback(const human_pose_msgs::HumanList humanList)
{
    person_info.clear();
    int humanNum = humanList.human_list.size();
    if(humanNum > 0)
    {
        for(int person=0; person<humanNum; ++person)
        {
            auto body_keypoints = humanList.human_list[person].body_key_points_prob;
            auto body_id = humanList.human_list[person].human_id;
            int action = humanList.human_list[person].action;
            int count = 0;
	        double prob_sum = 0.0;
	        for(int j=0;j < body_keypoints.size();j++)
	        {
	        	if(body_keypoints[j].p >= 0.0)
	            {
					prob_sum += body_keypoints[j].p;
					count++;
				}
			}
			double prob_eval = prob_sum/count;
			if(prob_eval < 0.6)
			{
				continue;
			}
            HumanPose::Pose_Info new_person;
            new_person.setLabel(body_id);   
            new_person.setAction(action); 
            new_person.setPose(humanList, person);
            new_person.trangulation(rot, trans);
            person_info.push_back(new_person);       
        }
    }
}


// 通过监听tf消息之间的关系获取相机姿态
void ImageProcess::waitForCameraPose(){
    tf::StampedTransform camera_pose;

    while(ros::ok()){
        try
        {
            tf_listener->waitForTransform("marker_0", "camera_base_" + to_string(_camera_index), ros::Time(0), ros::Duration(3.0));
            tf_listener->lookupTransform("marker_0", "camera_base_" + to_string(_camera_index), ros::Time(0), camera_pose);
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        Eigen::Quaterniond q(camera_pose.getRotation().getW(), camera_pose.getRotation().getX(), camera_pose.getRotation().getY(), camera_pose.getRotation().getZ());
        Eigen::Vector3d trans(camera_pose.getOrigin().getX(), camera_pose.getOrigin().getY(), camera_pose.getOrigin().getZ());
        this->rot = q.toRotationMatrix();
        this->trans = trans;
        break;
    }
    
	ROS_INFO("Camera_%d pose has listener sunccessfully!", this->_camera_index);
    delete tf_listener;
}


int recognizate_operator(ImageProcess& cam, Visualizate& vis){
    int index = -1;
    int count = 0;

    while(ros::ok()){
        vis.image = cam.image;
        try
        {
            for(int i=0; i<cam.person_info.size(); ++i){
                if(cam.person_info[i].action == 0){     //T-Pose
                    cam.person_info[i].is_opeator = true;
                }
            }
        }
        catch(const std::exception& e)
        {
            continue;
        }

        for(int i=0; i<cam.person_info.size(); ++i){
            if(cam.person_info[i].is_opeator == true){     //T-Pose
                ++count;
            }
            index = i;
        }

        bool pass = false;
        if(count >= 10){
            break;
        }

        vis.recognizating(cam.person_info);
        ros::spinOnce();
    }
    return index;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_person_recognition");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate rate(30);

    string calib_path = "/home/xuchengjun/catkin_ws/src/human_pose/calibration_data/003415165047";

    ImageProcess cam_a(1, "/kinect2_1", calib_path);
    cam_a.initCalibration(1.0);
    cam_a.waitForCameraPose();
    
    Visualizate vis(pose_pub);
    vis.addCameraInfo(cam_a.rot, cam_a.trans, cam_a.cameraMatrix);

    Track tr;
    
    //int operator_index = 0;
    int operator_index = recognizate_operator(cam_a, vis);
    int frame_index = 0;
    double time_total = 0.0;
    while (ros::ok())
    {
        frame_index++;
        auto start_time = chrono::system_clock::now();
        //cam_a.person_info[operator_index].label = 0;
        auto new_person_info = tr.tracking(cam_a.person_info, operator_index);

        vis.image = cam_a.image;
        vis.Publish3DPoses(cam_a.person_info);

        auto result = vis.show2DPose(cam_a.person_info, new_person_info,frame_index);
        ++frame_index;

        vis.Publish3DPosesTF(result);
        ros::spinOnce();
        if(frame_index > 3000){
            frame_index = 0;
            time_total = 0;
            //break;
        }

        // auto end_time = chrono::system_clock::now();
        // auto duration = duration_cast<microseconds>(end_time - start_time);
        // time_total += double(duration.count()) * microseconds::period::num / microseconds::period::den;
        //cout << "Time cost is: " << time_total/frame_index * 1000 << " ms" << endl;
        // rate.sleep();
    }
    
    return 0;
}
