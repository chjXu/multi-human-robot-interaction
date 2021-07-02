#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <opencv2/aruco.hpp>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <pthread.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <depth_registration.h>
#include <fstream>
#include <math.h>

#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include <geometry_msgs/PoseArray.h>
using namespace cv;
using namespace std;

typedef struct keypoints_with_prob
{
  double x;
  double y;
  double p;
}KeyPoints;

void visualize_depth(Mat& depth_image, Mat& depth_viz)
{
   if(!depth_image.empty())
   {
       depth_viz = Mat(depth_image.rows, depth_image.cols, CV_8UC3);
       for (int r = 0; r < depth_viz.rows; ++r)
        for (int c = 0; c < depth_viz.cols; ++c)
        {
            uint16_t depth = depth_image.at<uint16_t>(r, c);
            uint16_t level;
            uint8_t alpha;

            //sort depth information into different depth levels
            if (depth == 0)
                level = 0;
            else
                level = depth / 1000 + 1;
                alpha = (depth % 1000) / 4;

            switch(level)
            {
                case(1):

                    depth_viz.at<Vec3b>(r, c) = Vec3b(0, 0, alpha);
                    break;
                case(2):
                    depth_viz.at<Vec3b>(r, c) = Vec3b(0, alpha, 255);
                    break;
                case(3):
                    depth_viz.at<Vec3b>(r, c) = Vec3b(0, 255, 255-alpha);
                    break;
                case(4):
                    depth_viz.at<Vec3b>(r, c) = Vec3b(alpha, 255, 0);
                    break;
                case(5):
                    depth_viz.at<Vec3b>(r, c) = Vec3b(255, 255-alpha, 0);
                    break;
                default:
                    depth_viz.at<Vec3b>(r, c) = Vec3b(0, 0, 0);
                    break;
           }

        }
   }
}


class ImageProcessor
{
  public:
    ImageProcessor(string sensor, bool isCalibrationMode):it(nh), priv_nh(ros::NodeHandle("~")), sizeColor(1920, 1080), left_key_points_center(480,260), right_key_points_center(480,260), left_ROI(240), right_ROI(240){
      color_mat = Mat(sizeColor, CV_8UC3);
      depth_mat = Mat(sizeColor, CV_16UC1);



      string color_topic = "/" + sensor + "/hd/image_color_rect";
      string depth_topic = "/" + sensor + "/hd/image_depth_rect";
      sub = it.subscribe(color_topic.c_str(), 1,&ImageProcessor::imageCallback,this);
      sub2 = it.subscribe(depth_topic.c_str(),1,&ImageProcessor::depthimageCallback,this);

      marker_0_sum_count = 0;

      isCamBaseTransformAvailable = false;
      isCamHumanTransformAvailable = false;
      firstDepth = true;
      calibrationMode = isCalibrationMode;
      if(!calibrationMode)
      {
        string pose_solution_path = "/home/agent/luk_ws/robot_pose/solution_dobot";
        loadRobotPoseFile(pose_solution_path);
      }
    }
    bool getImage(Mat&);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthimageCallback(const sensor_msgs::ImageConstPtr& msgdepth);
    void loadCalibrationFiles();
    void sendMarkerTF(vector<Vec3d>& marker_trans, vector<Vec3d>& marker_rot, vector<int>& ids);
    void sendCameraTF(vector<Vec3d>& marker_trans, vector<Vec3d>& marker_rot, vector<int>& ids);
    void sendMarkerTf(vector<Point3f>& marker_position, vector<int>& ids);
    void getWorldCoordinate(Point2f& image_cord, Point3f& cord);
    void getWorldCoordinate(Mat& depth_image, Point2f& image_cord, Point3f& cord);
    void calculateRobotPose(vector<Point>& joint_image_cords, vector<Point3f>& joint_3d_cords);
    void getImageCoordinate(Point3f& world_cord, Point& image_cord);
    void drawRobotJoints(Mat& image, vector<Point>& joint_image_cords);
    void linkToRobotTf();

    void human_joint_callback(const geometry_msgs::PoseArray& poses);
    void calculateHumanPose(vector<Point>& joint_image_cords, vector<Point3f>& joint_3d_cords);
    void draw_human_pose(Mat& image, vector<Point>& human_joints);
    //void getFivemarkerWorldCoordinate(vector<vector<Point2f>>& corners, vector<int>& ids, vector<Point2f>& marker_center, vector<Point3f>& world_cord);
    bool removeRobotImage(Mat& image,vector<Point>& robot_image_pos, vector<Point3f>& robot_image_3d_pos, Mat& );
    bool drawPointPiexs(Mat& image,vector<Point>& robot_image_pos, vector<Point3f>& robot_image_3d_pos, Mat&, int n);

    static void* publishRobotThread(void *arg);
    void startRobotThread();
    void stopRobotThread();

    void publishSplicedImage();
    static void* publishSplicedImageThread(void *arg);
    void startSplicedThread();
    void stopSplicedThread();

    void human_keypoints_callback(openpose_ros_msgs::OpenPoseHumanList keypoints);
    void drawKeyPoints(Mat& image, vector<KeyPoints>& points);

    void extractFeature();
  private:
    ros::NodeHandle nh, priv_nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Subscriber sub2;
    image_transport::Subscriber subTF;
    image_transport::Publisher spliced_image_pub;

    ros::Subscriber human_joint_sub;
    ros::Subscriber human_keypoints_sub;
    cv::Mat distortion_color;
    cv::Mat cameraMatrix_color_clipped;
    cv::Mat camera_matrix;
    Point2f averagecorners;

    //vector<Point> robot_image_pos;
    //vector<Point3f>robot_image_3d_pos;

    Vec3d marker_0_tvecs_sum;
    Vec3d marker_0_rvecs_sum;
    int marker_0_sum_count;

    cv::Mat color_mat;
    cv::Mat color_mat_copy;
    cv::Mat depth_mat;
    cv::Mat depth_debug;


    tf::TransformListener robot_pose_listener;
    tf::StampedTransform base_cam_transform;
    bool isCamBaseTransformAvailable;
    bool isCamHumanTransformAvailable;
    vector<std::string> joint_names;
    vector<std::string> human_joint_names;
    Size sizeColor;
    float fx, fy, cx, cy;
    bool calibrationMode;
    bool firstDepth;

    vector<Point> human_joint_pos;

    tf::Transform robot_pose_tansform;
    void loadRobotPoseFile(string);

    pthread_t id1, id2;
    vector<KeyPoints> left_key_points;
    Point left_key_points_center;

    vector<KeyPoints> right_key_points;
    Point right_key_points_center;
    int left_ROI, right_ROI;

};


void ImageProcessor::extractFeature()
{

    static tf::TransformBroadcaster tracking_position_broadcaster;

    int iLowH = 30;
    int iHighH = 75;

    int iLowS = 90;
    int iHighS = 255;

    int iLowV = 70;
    int iHighV = 255;
    if(!color_mat_copy.empty() && ! depth_debug.empty())
    {
      tf::StampedTransform robot_transform;

      bool RobotPoseAvailable = true;
      try
      {
          robot_pose_listener.lookupTransform("camera_base", "ur_base", ros::Time(0), robot_transform);
      }

      catch(tf::TransformException ex)
      {
          //ROS_ERROR("%s", ex.what());
          RobotPoseAvailable = false;
          //return;
      }
      Mat color_HSV;
      vector<Mat> hsvSplit;
      cvtColor(color_mat_copy, color_HSV, COLOR_BGR2HSV);

      split(color_HSV, hsvSplit);
      equalizeHist(hsvSplit[2], hsvSplit[2]);
      merge(hsvSplit, color_HSV);

      Mat imgThresholded;

      inRange(color_HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

      Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
      morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

      morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

      Mat dstImg = color_mat_copy.clone();

      //cvtColor(imgThresholded, imgThresholded, CV_HSV2GRAY);
      threshold(imgThresholded, imgThresholded, 100, 255, CV_THRESH_BINARY);

      vector<vector<Point>> contours;
      vector<Vec4i> hierarcy;
      findContours(imgThresholded, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      vector<Rect> boundRect(contours.size());
      vector<RotatedRect> box(contours.size());
      Point2f rect[4];
      ostringstream cord_text;

      for(int i = 0; i < contours.size(); ++i){
          box[i] = minAreaRect(Mat(contours[i]));
          boundRect[i] = boundingRect(Mat(contours[i]));
          //circle(dstImg, Point(box[i].center.x, box[i].center.y), 5, Scalar(255, 0, 0), -1, 8);
          box[i].points(rect);
          //rectangle(dstImg, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
          /**
          for(int j=0; j < 4; ++j)
          {
              line(dstImg, rect[j], rect[(j+1)%4], Scalar(0, 0, 255), 2, 8);
          }
          **/
          Point2f image_cord((float)box[i].center.x, (float)box[i].center.y);
          Point3f location;
          getWorldCoordinate(depth_debug, image_cord, location);

          if(RobotPoseAvailable){
              Point3f robot_location(robot_transform.getOrigin().x(), robot_transform.getOrigin().y(), robot_transform.getOrigin().z());
              Point3f distance;
              distance = robot_location - location;
              if (norm(distance) < 1){
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(location.x, location.y, location.z));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                tracking_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base", "tracked"));


                circle(dstImg, Point(box[i].center.x, box[i].center.y), 5, Scalar(255, 0, 0), -1, 8);
                for(int j=0; j < 4; ++j)
                {
                    line(dstImg, rect[j], rect[(j+1)%4], Scalar(0, 0, 255), 2, 8);
                }
                cord_text.str("");
                cord_text << "position:" << " at" << '(' << location.x << ',' << location.y << ',' << location.z << ')';
                putText(dstImg, cord_text.str(), Point(box[i].center.x + 40, box[i].center.y + 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255, 0));
              }
          }



      }



      imshow("tracked", dstImg);
      imshow("threshold", imgThresholded);
      waitKey(30);
    }

}


void ImageProcessor::drawRobotJoints(Mat& image, vector<Point>& joint_image_cords)
{

    //cout << "x: " << joint_image_cords[0].x << "\n";
    //cout << "y: " << joint_image_cords[0].y << "\n";
    circle(image, joint_image_cords[0], 3, Scalar(0, 255, 0), -1, 8);
    //draw joints on image
    for(int i = 1; i < (joint_image_cords.size()); i++)
    {
        circle(image, joint_image_cords[i], 3, Scalar(0, 255, 0), -1, 8);
        line(image,joint_image_cords[i-1],joint_image_cords[i], Scalar(0, 255, 255), 2);
    }
}
void ImageProcessor::getImageCoordinate(Point3f& world_cord, Point& image_cord)
{
    image_cord.x = (int)(world_cord.x * fx / world_cord.z + cx);
    image_cord.y = (int)(world_cord.y * fy / world_cord.z + cy);
}

void ImageProcessor::calculateRobotPose(vector<Point>& joint_image_cords, vector<Point3f>& joint_3d_cords)
{
  //tf::TransformListener robot_pose_listener;
    string robot_reference_frame;
    if (calibrationMode)
    {
        robot_reference_frame = "camera_base_rect";
    }
    else
    {
        robot_reference_frame = "camera_base";
    }


    tf::StampedTransform joint_transforms;
    tf::StampedTransform cam_base_transform;
    try
    {
        robot_pose_listener.lookupTransform(robot_reference_frame.c_str(), "base_link", ros::Time(0), cam_base_transform);
    }

    catch(tf::TransformException ex)
    {
        //ROS_ERROR("%s", ex.what());
        isCamBaseTransformAvailable = false;
        return;
    }

    isCamBaseTransformAvailable = true;
    Point3f base_location(cam_base_transform.getOrigin().x(), cam_base_transform.getOrigin().y(), cam_base_transform.getOrigin().z());
    Point base_image_cord;
    getImageCoordinate(base_location, base_image_cord);
    //circle(color_mat, base_image_cord, 2, Scalar(0, 255, 255), -1, 8);

    ostringstream cord_text;
    cord_text.str("");
    cord_text << "base_position:" << " at" << '(' << base_location.x << ',' << base_location.y << ',' << base_location.z << ')';
    putText(color_mat, cord_text.str(), Point(20,400), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255, 0));

    //vector<Point3f> joint_3d_cords;
    joint_3d_cords.push_back(base_location);
    //vector<Point> joint_image_cords;
    joint_image_cords.push_back(base_image_cord);
    for(int i = 0; i < joint_names.size(); i++)
    {
        try
        {
            robot_pose_listener.lookupTransform(robot_reference_frame.c_str(), joint_names[i], ros::Time(0), joint_transforms);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            continue;
        }
        Point3f location(joint_transforms.getOrigin().x(), joint_transforms.getOrigin().y(), joint_transforms.getOrigin().z());
        joint_3d_cords.push_back(location);
        Point joint_image_cord;
        getImageCoordinate(location, joint_image_cord);
        joint_image_cords.push_back(joint_image_cord);
        //ROS_INFO("Robot Pose Get!");

    }

}
void ImageProcessor::loadCalibrationFiles()
{

    cv::FileStorage fs;
    string calib_path = "/home/agent/catkin_ws/src/camera_wrapper/calibration_data/003415165047/";
    cv::Mat cameraMatrix_color;


  if(fs.open(calib_path + "calib_color.yaml", cv::FileStorage::READ))
  {
    fs["cameraMatrix"] >> cameraMatrix_color;
    cameraMatrix_color_clipped = cameraMatrix_color.clone();
    cameraMatrix_color_clipped.at<double>(0, 0) /= 1;
    cameraMatrix_color_clipped.at<double>(1, 1) /= 1;
    cameraMatrix_color_clipped.at<double>(0, 2) /= 1;
    cameraMatrix_color_clipped.at<double>(1, 2) /= 1;
    fx = cameraMatrix_color_clipped.at<double>(0, 0);
    fy = cameraMatrix_color_clipped.at<double>(1, 1);
    cx = cameraMatrix_color_clipped.at<double>(0, 2);
    cy = cameraMatrix_color_clipped.at<double>(1, 2);

    fs["distortionCoefficients"] >> distortion_color;
    cout << "color matrix load success"<< endl;
    fs.release();


  }
  else
  {
    cout << "No calibration file: calib_color.yalm, using default calibration setting" << endl;
    cameraMatrix_color_clipped = cv::Mat::eye(3, 3, CV_64F);
    distortion_color = cv::Mat::zeros(1, 5, CV_64F);


  }


}
void ImageProcessor::getWorldCoordinate(Point2f& image_cord, Point3f& cord)
{

    if(!color_mat.empty() && !depth_mat.empty() && image_cord.x < sizeColor.width && image_cord.y < sizeColor.height)
    {

        uint16_t d = depth_mat.at<uint16_t>(image_cord);

        cord.z = float(d) * 0.001f;
        //printf("%.4f\n", cord.z);
        cord.x = ((image_cord.x - cx) * cord.z) / fx;
        cord.y = ((image_cord.y - cy) * cord.z) / fy;
    }
}

void ImageProcessor::getWorldCoordinate(Mat& depth_image, Point2f& image_cord, Point3f& cord)
{

    if(!depth_image.empty() && image_cord.x < sizeColor.width && image_cord.y < sizeColor.height)
    {

        uint16_t d = depth_image.at<uint16_t>(image_cord);

        cord.z = float(d) * 0.001f;
        //printf("%.4f\n", cord.z);
        cord.x = ((image_cord.x - cx) * cord.z) / fx;
        cord.y = ((image_cord.y - cy) * cord.z) / fy;
    }
}

void ImageProcessor::depthimageCallback(const sensor_msgs::ImageConstPtr& msgdepth)
{

	try
    {
      vector<int> markerfiveids(1,5);

    	depth_mat = cv_bridge::toCvShare(msgdepth, "16UC1")->image;
      depth_debug = depth_mat.clone();

      Mat depth_viz;
      visualize_depth(depth_mat, depth_viz);
      cv::imshow("depth", depth_viz);
    	//cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    	//detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       // cv::Mat displyImg = cv_bridge::toCvShare(msgdepth, "mono16")->image.clone();
        //Point2f marker_center
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("Could not convert from '%s' to 'mono16'.", msgdepth->encoding.c_str());
    }
}

void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    bool drawRobot;
    nh.param("drawRobot", drawRobot, false);
    try
    {
      //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      color_mat = cv_bridge::toCvShare(msg, "bgr8")->image;
      color_mat_copy = color_mat.clone();
      cv::Mat displyImg = color_mat.clone();

      vector<Point> joint_image_cords;
      vector<Point3f> joint_3d_cords;
      //namedWindow("Color Frame");
      vector<Point> human_image_cords;
      vector<Point3f> human_3d_cords;

      if(!color_mat.empty() && !depth_mat.empty())
      {
        calculateRobotPose(joint_image_cords, joint_3d_cords);
        if(!joint_image_cords.empty())
        {
          //Mat depth_debug;
          //depth_debug = depth_mat.clone();
          //if(!removeRobotImage(displyImg, joint_image_cords, joint_3d_cords, depth_debug))
            //imshow("origin depth", depth_mat);
          if(drawRobot)
            drawRobotJoints(displyImg,joint_image_cords);
        }
      }



      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

      Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
      detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
      //detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
      //detectorParams->errorCorrectionRate = 0.1;
      //detectorParams->minOtsuStdDev = 10;

      //cv::Mat displyImg = color_mat.clone();
      //flip(displyImg,displyImg,1);
      std::vector< int > ids;
      std::vector< std::vector< cv::Point2f > > corners, rejected;

      //std::vector<cv::Point3f> world_cord;
      //  vector< Vec3d > rvecs, tvecs;
      // detect markers and estimate pose
      cv::aruco::detectMarkers(color_mat, dictionary, corners, ids, detectorParams);
      /***
      for(int j = 0; j < ids.size(); j++)
      {
        if(ids[j] == 5)
        {
          averagecorners.x = 0.f;
          averagecorners.y = 0.f;
          for (int i = 0; i < corners[j].size(); i++)
          {
       		    averagecorners = averagecorners + corners[j][i];
          }
          averagecorners /= 4.0;
        }
      }
      ***/

      //printf("%d\n",ids.size());
      if (ids.size() > 0)
      {
        cv::aruco::drawDetectedMarkers(displyImg, corners, ids);
        //aruco::drawDetectedMarkers(displyImg, rejected, noArray(), Scalar(100, 0, 255));
        std::vector<cv::Vec3d> rvecs_bigger,tvecs_bigger;
        std::vector<cv::Vec3d> rvecs,tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners,0.162f,cameraMatrix_color_clipped,distortion_color,rvecs_bigger,tvecs_bigger);
        cv::aruco::estimatePoseSingleMarkers(corners,0.162f,cameraMatrix_color_clipped,distortion_color,rvecs,tvecs);
        for(int i = 0; i<ids.size(); i++)
          {

             cv::aruco::drawAxis(displyImg,cameraMatrix_color_clipped,distortion_color,rvecs[i],tvecs[i],0.1);
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
             }
             //sendMarkerTF(tvecs, rvecs, ids);
             sendCameraTF(tvecs_bigger, rvecs_bigger, ids);

          }
            //sendMarkerTF(tvecs, rvecs, ids);
      }

      cv::aruco::drawDetectedMarkers(displyImg, rejected, noArray(), Scalar(100, 0, 255));
      cv::imshow("Markers",displyImg);
      cv::waitKey(30);
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
      if(ids[i] == 5)
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
        marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base", oss.str()));
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
        marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base", oss.str()));
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
    robot_pose_broadcaster.sendTransform(tf::StampedTransform(robot_pose_tansform, ros::Time::now(), "marker_0", "ur_base"));
}

bool ImageProcessor::drawPointPiexs(Mat& image,vector<Point>& robot_image_pos,
                                      vector<Point3f>& robot_image_3d_pos,
                                      Mat& depth_image,int numb)
{
  float depth;
  int count = 0;
  for (int i = robot_image_pos[numb].x - 30; i < robot_image_pos[numb].x + 30; i++)
  {
    for (int j = robot_image_pos[numb].y - 30; j < robot_image_pos[numb].y + 30; j++)
    {
        depth = (float)depth_image.at<uint16_t>(j, i) / 1000;
        //std::cout << depth << '\n';
        if ((depth < robot_image_3d_pos[numb].z + 0.1) && (depth > robot_image_3d_pos[numb].z - 0.1))
        {
           image.at<Vec3b>(j,i)[0] = 200;
           image.at<Vec3b>(j,i)[1] = 200;
           image.at<Vec3b>(j,i)[2] = 255;
           count++;
        }
    }
  }
  if (count > 0)
    return true;
  else
    return false;
}

void* ImageProcessor::publishRobotThread(void *arg)
{
    ImageProcessor *ptr = (ImageProcessor *) arg;
    ros::Rate rate(10);
    while(1)
    {
        ptr->linkToRobotTf();

        pthread_testcancel(); //thread cancel point

        //ros::spinOnce();

        rate.sleep();
    }
}

void ImageProcessor::startRobotThread()
{
    int ret = pthread_create(&id1, NULL, publishRobotThread, (void*)this);
}
void ImageProcessor::stopRobotThread()
{
    int ret = pthread_cancel(id1);
}

bool ImageProcessor::getImage(Mat& image)
{
  if(!color_mat.empty()){
    image = color_mat.clone();
    return true;
  }
  else
    return false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  //cv::namedWindow("view");
  cv::startWindowThread();
  bool isCalibrationMode = true;
  string sensor = "kinect_1";
  if(argc > 1)
  {
    for(size_t i = 1; i < (size_t)argc; ++i)
    {
        printf("arg :%s\n", argv[i]);
        string arg = argv[i];
        if (arg == "false")
        {
            isCalibrationMode = false;
            ROS_INFO("calibrationMode disabled\n");
        }
        else
        {
          sensor = arg;
          ROS_INFO("Subscribing to %s ns", sensor.c_str());
        }
    }
  }
  ImageProcessor img_processor(sensor, isCalibrationMode);
  img_processor.loadCalibrationFiles();
  //ros::AsyncSpinner spinner(2); //use 2 threads
  //spinner.start();
  ros::Rate rate(100);
  if(!isCalibrationMode){
    img_processor.startRobotThread();
  }
  //img_processor.startSplicedThread();
  //ros::MultiThreadedSpinner spinner(2);
  //spinner.spin();
  //ros::waitForShutdown();
  //spinner.stop();

  while(ros::ok())
  {
    //if(!isCalibrationMode)
      //img_processor.linkToRobotTf();
    ros::spinOnce();
    img_processor.extractFeature();
    rate.sleep();
  }

  if(!isCalibrationMode){
    img_processor.stopRobotThread();
  }
  //img_processor.stopSplicedThread();
  /***

  ***/
  return 0;

}
