#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//#include <OpenNI.h>
#include <opencv2/aruco.hpp>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <pthread.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <depth_registration.h>
#include <fstream>
#include <math.h>

//#include <openpose_ros_msgs/OpenPoseHumanList.h>
//#include <openpose_ros_msgs/PointWithProb.h>

//#include <inertial_poser/ROI_Package.h>
//#include <inertial_poser/UpperBodyKeyPoints.h>
//#include <inertial_poser/KeyPoint.h>

#include <geometry_msgs/PoseArray.h>
using namespace cv;
using namespace std;


class ImageProcessor
{
  public:
    ImageProcessor(int camera_index, string nspace, string calib_path):it(nh), ns(nspace), calib_path(calib_path), camera_index(camera_index)
    {
      string color_topic = ns + "/hd/image_color";
      // if(nspace == "realSense"){
      //   color_topic = nspace + "/color/image_raw";
      // }

      color_sub = it.subscribe(color_topic, 1,&ImageProcessor::imageCallback,this);
      cout << "topic: " << color_topic << endl;
      initCalibration(1.0);
      pthread_mutex_init (&mutex, NULL);
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback_simple(const sensor_msgs::ImageConstPtr& msg);
    void loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distortion, double scale);
    void initCalibration(double);
    void sendMarkerTF(vector<Vec3d>& marker_trans, vector<Vec3d>& marker_rot, vector<int>& ids);
    void sendCameraTF(vector<Vec3d>& marker_trans, vector<Vec3d>& marker_rot, vector<int>& ids);
    void sendMarkerTf(vector<Point3f>& marker_position, vector<int>& ids);
    void getWorldCoordinate(Point2f& image_cord, Point3f& cord, cv::Mat&);
    void getImageCoordinate(Point3f& world_cord, Point& image_cord, cv::Mat&);
    void linkToRobotTf();

    void loadRobotPoseFile(string);
    static void* publishRobotThread(void *arg);
    void startRobotThread();
    void stopRobotThread();

    void SwitchTopic();
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber color_sub;

    cv::Mat distortion;
    cv::Mat cameraMatrix;
    Point2f averagecorners;


    Vec3d marker_0_tvecs_sum;
    Vec3d marker_0_rvecs_sum;
    int marker_0_sum_count;


    cv::Mat color_mat;
    cv::Mat displyMat;


    tf::TransformListener robot_pose_listener;
    tf::StampedTransform base_cam_transform;
    bool isCamBaseTransformAvailable;
    bool isCamHumanTransformAvailable;
    bool calibrationMode;

    tf::Transform robot_pose_tansform;
    

    pthread_t id1, id2;
    pthread_mutex_t mutex;
    string ns;

    bool rect;

    string calib_path;

    int camera_index;

};

// void ImageProcessor::SwitchTopic(){

//   initCalibration(0.5);
//   string color_topic = ns + "/qhd/image_color";
//   if(rect){
//     color_topic = color_topic + "_rect";
//   }
//   color_sub = it.subscribe(color_topic.c_str(), 1,&ImageProcessor::imageCallback_simple, this);

// }

void ImageProcessor::initCalibration(double scale){
    loadCalibrationFiles(calib_path, cameraMatrix, distortion, scale);
}

void ImageProcessor::getImageCoordinate(Point3f& world_cord, Point& image_cord, Mat& cameraMatrix)
{
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    image_cord.x = (int)(world_cord.x * fx / world_cord.z + cx);
    image_cord.y = (int)(world_cord.y * fy / world_cord.z + cy);
}

void ImageProcessor::loadCalibrationFiles(string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distortion, double scale)
{

    cv::FileStorage fs;

    cv::Mat cameraMatrix_origin;


  if(fs.open(calib_path + "/calib_color.yaml", cv::FileStorage::READ))
  {
    fs["cameraMatrix"] >> cameraMatrix_origin;
    cameraMatrix = cameraMatrix_origin.clone();
    cameraMatrix.at<double>(0, 0) *= scale;
    cameraMatrix.at<double>(1, 1) *= scale;
    cameraMatrix.at<double>(0, 2) *= scale;
    cameraMatrix.at<double>(1, 2) *= scale;

    distortion= cv::Mat::zeros(1, 5, CV_64F);

    //fs["distortionCoefficients"] >> distortion_color;
    cout << "color matrix load success"<< endl;
    fs.release();
  }
  else
  {
    cout << "No calibration file: calib_color.yalm, using default calibration setting" << endl;
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distortion = cv::Mat::zeros(1, 5, CV_64F);
  }
}


void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      Mat color_tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
      pthread_mutex_lock(&mutex);
      color_mat = color_tmp.clone();
      pthread_mutex_unlock(&mutex);
      Mat displyImg = color_mat.clone();

      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

      Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
      detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
      
      std::vector< int > ids;
      std::vector< std::vector< cv::Point2f > > corners, rejected;

      //  vector< Vec3d > rvecs, tvecs;
      // detect markers and estimate pose
      cv::aruco::detectMarkers(color_mat, dictionary, corners, ids);
      
      if (ids.size() > 0)
      {
        cv::aruco::drawDetectedMarkers(displyImg, corners, ids);
        //aruco::drawDetectedMarkers(displyImg, rejected, noArray(), Scalar(100, 0, 255));
        std::vector<cv::Vec3d> rvecs,tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners,0.147f, cameraMatrix, distortion,rvecs,tvecs);
        
        bool has_Marker_0 = false;
        for(int i = 0; i<ids.size(); i++)
          {

            //cv::aruco::drawAxis(displyImg,cameraMatrix,distortion,rvecs[i],tvecs[i],0.1);
             if (ids[i] == 0)
             {
                 if(marker_0_sum_count == 0){
                     marker_0_rvecs_sum = rvecs[i];
                     marker_0_tvecs_sum = tvecs[i];
                     marker_0_sum_count = 1;
                 }
                 Vec3d t_diff = marker_0_tvecs_sum / marker_0_sum_count - tvecs[i];
                 Vec3d r_diff = marker_0_rvecs_sum / marker_0_sum_count - rvecs[i];
                 if ((norm(t_diff) < 0.03 && norm(r_diff) < 0.1 )){
                   if(marker_0_sum_count < 100){
                     marker_0_rvecs_sum += rvecs[i];
                     marker_0_tvecs_sum += tvecs[i];
                     marker_0_sum_count ++;
                   }
                   else{
                   tvecs[i] = marker_0_tvecs_sum / marker_0_sum_count;
                   rvecs[i] = marker_0_rvecs_sum / marker_0_sum_count;
                   }
                 }
                 else{
                   marker_0_rvecs_sum = rvecs[i];
                   marker_0_tvecs_sum = tvecs[i];
                   marker_0_sum_count = 1;
                 }
                 sendCameraTF(tvecs, rvecs, ids);
                 has_Marker_0 = true;
             }
          }
          if(!has_Marker_0)
            if(marker_0_sum_count > 0)
            {
              ids.push_back(0);
              rvecs.push_back(marker_0_rvecs_sum / marker_0_sum_count);
              tvecs.push_back(marker_0_tvecs_sum / marker_0_sum_count);
              sendCameraTF(tvecs, rvecs, ids);
            }

        sendMarkerTF(tvecs, rvecs, ids);
      }

      else if(marker_0_sum_count > 0)
      {
          std::vector<cv::Vec3d> rvecs,tvecs;
          ids.push_back(0);
          rvecs.push_back(marker_0_rvecs_sum / marker_0_sum_count);
          tvecs.push_back(marker_0_tvecs_sum / marker_0_sum_count);
          sendCameraTF(tvecs, rvecs, ids);
      }
      //cv::aruco::drawDetectedMarkers(displyImg, rejected, noArray(), Scalar(100, 0, 255));
      // cv::resize(displyImg, displyImg, Size(960, 540));
      // displyMat = displyImg.clone();
      // cv::imshow(this->ns, displyImg);
      // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageProcessor::imageCallback_simple(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      Mat color_tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
      pthread_mutex_lock(&mutex);
      color_mat = color_tmp.clone();
      pthread_mutex_unlock(&mutex);

      vector<Point> human_image_cords;
      vector<Point3f> human_3d_cords;

      std::vector<int> ids;
      std::vector<cv::Vec3d> rvecs,tvecs;
      if(marker_0_sum_count > 0)
      {
              ids.push_back(0);
              rvecs.push_back(marker_0_rvecs_sum / marker_0_sum_count);
              tvecs.push_back(marker_0_tvecs_sum / marker_0_sum_count);
              sendCameraTF(tvecs, rvecs, ids);
      }
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageProcessor::sendMarkerTf(vector<Point3f>& marker_position, vector<int>& ids)
{
    static tf::TransformBroadcaster marker_position_broadcaster;
    for(int i = 0; i < marker_position.size(); i++)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(marker_position[i].x, marker_position[i].y, marker_position[i].z));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        ostringstream oss;
        oss << "marker_" << ids[i];
        marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base", oss.str()));
    }
}

void ImageProcessor::sendMarkerTF(vector<Vec3d>& marker_trans, vector<Vec3d>& marker_rot, vector<int>& ids)
{
    Mat rot(3, 3, CV_64FC1);
    Mat rot_to_ros(3, 3, CV_64FC1);
    rot_to_ros.at<double>(0,0) = -1.0;
    rot_to_ros.at<double>(0,1) = 0.0;
    rot_to_ros.at<double>(0,2) = 0.0;
    rot_to_ros.at<double>(1,0) = 0.0;
    rot_to_ros.at<double>(1,1) = 0.0;
    rot_to_ros.at<double>(1,2) = 1.0;
    rot_to_ros.at<double>(2,0) = 0.0;
    rot_to_ros.at<double>(2,1) = 1.0;
    rot_to_ros.at<double>(2,2) = 0.0;

    static tf::TransformBroadcaster marker_position_broadcaster;
    for(int i = 0; i < ids.size(); i++)
    {
      if(ids[i] == 10)
      {

        cv::Rodrigues(marker_rot[i], rot);
        rot.convertTo(rot, CV_64FC1);


        tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                             rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                             rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

        tf::Vector3 tf_trans(marker_trans[i][0], marker_trans[i][1], marker_trans[i][2]);
        tf::Transform transform(tf_rot, tf_trans);
        ostringstream oss;
        oss << "marker_" << ids[i];
        marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "prime_camera_base", oss.str()));
      }
    }
}

void ImageProcessor::sendCameraTF(vector<Vec3d>& marker_trans, vector<Vec3d>& marker_rot, vector<int>& ids)
{
    Mat rot(3, 3, CV_64FC1);
    Mat rot_to_ros(3, 3, CV_64FC1);
    rot_to_ros.at<double>(0,0) = -1.0;
    rot_to_ros.at<double>(0,1) = 0.0;
    rot_to_ros.at<double>(0,2) = 0.0;
    rot_to_ros.at<double>(1,0) = 0.0;
    rot_to_ros.at<double>(1,1) = 0.0;
    rot_to_ros.at<double>(1,2) = 1.0;
    rot_to_ros.at<double>(2,0) = 0.0;
    rot_to_ros.at<double>(2,1) = 1.0;
    rot_to_ros.at<double>(2,2) = 0.0;

    char camera_frame_name[50];
    sprintf(camera_frame_name, "camera_base_%d", camera_index);

    static tf::TransformBroadcaster marker_position_broadcaster;
    for(int i = 0; i < ids.size(); i++)
    {
      if(ids[i] == 0)
      {

        cv::Rodrigues(marker_rot[i], rot);
        rot.convertTo(rot, CV_64FC1);


        tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                             rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                             rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

        tf::Vector3 tf_trans(marker_trans[i][0], marker_trans[i][1], marker_trans[i][2]);
        tf::Transform transform(tf_rot, tf_trans);
        ostringstream oss;
        oss << "marker_" << ids[i];
        marker_position_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), oss.str(), camera_frame_name));
      }
    }
}

void ImageProcessor::loadRobotPoseFile(string filename)
{
    ifstream inStream(filename);
    if (inStream)
    {
        vector<double> solution;
        int i = 0;
        while(!inStream.eof())
        {
            double in;
            inStream >> in;
            solution.push_back(in);
            i++;
        }
        vector<double>::iterator it = solution.end() - 1;
        solution.erase(it);
        for(int i = 0; i < solution.size(); i++)
        {
            cout << solution[i] << endl;
        }
        if(solution.size() != 10)
        {
            ROS_ERROR("Solution file invalid!");
            return;
        }
        robot_pose_tansform.setOrigin(tf::Vector3(solution[0] + solution[7], solution[1] + solution[8], solution[2] + solution[9]));
        //robot_pose_tansform.setOrigin(-tf::Vector3(solution[0], solution[1], solution[2]));
        tf::Quaternion q(solution[3], solution[4], solution[5], solution[6]);
        tf::Quaternion trans;
	      //这儿设置RPY(0,0,0)，再与q相乘是什么操作？
        trans.setRPY(0,0,0);
        q = q * trans;
        //q = q.inverse();
        robot_pose_tansform.setRotation(q);

        cout << "x: " << robot_pose_tansform.getRotation().x() << endl;
        cout << "y: " << robot_pose_tansform.getRotation().y() << endl;
        cout << "z: " << robot_pose_tansform.getRotation().z() << endl;
        cout << "w: " << robot_pose_tansform.getRotation().w() << endl;

    }
}
void ImageProcessor::linkToRobotTf()
{
    static tf::TransformBroadcaster robot_pose_broadcaster;

    robot_pose_broadcaster.sendTransform(tf::StampedTransform(robot_pose_tansform, ros::Time::now(), "marker_0", "world"));
}


void ImageProcessor::startRobotThread()
{
    int ret = pthread_create(&id1, NULL, publishRobotThread, (void*)this);
}
void ImageProcessor::stopRobotThread()
{
    int ret = pthread_cancel(id1);
}

void* ImageProcessor::publishRobotThread(void *arg)
{
    ImageProcessor *ptr = (ImageProcessor *) arg;
    ros::Rate rate(10);
    while(ros::ok())
    {
        ptr->linkToRobotTf();

        pthread_testcancel(); //thread cancel point

        ros::spinOnce();

        rate.sleep();
    }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  cv::startWindowThread();
  bool isCalibrationMode = false;
  bool sendSplicedImage = false;
  
  string calib_path_1;
  string calib_path_2;
  nh.getParam("cam1_params", calib_path_1);
  nh.getParam("cam2_params", calib_path_2);
  nh.getParam("calibMode", isCalibrationMode);
  string calib_path_3 = "/home/xuchengjun/catkin_ws/src/cv_camera/calibration_data";
  ImageProcessor img_processor_1(1, "/kinect2_1", calib_path_1);
  //ImageProcessor img_processor_2(2, "/kinect2_2", calib_path_2);
  //ImageProcessor img_processor_3(3, "/camera_3", calib_path_3, rect, isCalibrationMode);
  
  //img_processor_1.initCalibration(1.0);
  //img_processor_2.initCalibration(1.0);
  //img_processor_3.initCalibration(1.0);
  ros::Rate rate(30);
  if(!isCalibrationMode){
    string pose_solution_path = "/home/xuchengjun/catkin_ws/src/camera_wrapper/calibration_data/solution_20180814";
    img_processor_1.loadRobotPoseFile(pose_solution_path);  
    ROS_INFO("Start robot thread...");
    img_processor_1.startRobotThread();
  }
  int time_counter = 0;
  while(ros::ok() && time_counter < 30)
  {
    ros::spinOnce();
    time_counter ++;
    rate.sleep();
  }

  //img_processor_1.SwitchTopic();
  //img_processor_2.SwitchTopic();
  //img_processor_3.SwitchTopic();
  ros::spin();
  if(!isCalibrationMode){
    img_processor_1.stopRobotThread();
  }

  return 0;
}
