#include <iostream>
#include <ostream>
#include <string>
#include <vector>
#include <queue>

// ros ------------------------------------------------- 
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// tf --------------------------------------------------
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
// Eigen -----------------------------------------------
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// pose/human ------------------------------------------
#include <human_pose_msgs/Human.h>
#include <human_pose_msgs/HumanList.h>
#include <human_pose_msgs/PointWithProb.h>
// point publish ---------------------------------------
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <human_pose/pose.h>
#include <human_pose/associate.h>
#include <human_pose/track.h>

#define waittime 3
#define N 19
const int pairNum=18;

int bodyEages[pairNum][2] = {{0,1},{0,2},{0,9},{0,3},{9,10},{10,11},{3,4},{4,5},{2,12},{12,13},
                        {13,14},{2,6},{6,7},{7,8},{1,15},
                        {15,17},{1,16},{16,18}};

using namespace std;
using namespace Eigen;


class Camera
{
public:
    vector<HumanPose::Pose> SourecePose; //此容器既要接受callback的返回值，也要更新到世界坐标系下(可视化有问题)
    vector<HumanPose::Pose> TransPose;
public:
    Camera(int camera_id, ros::Publisher &marker_pub):camera_id(camera_id), marker_pub(&marker_pub)
    {
        this->m_line.header.frame_id = this->text.header.frame_id = "marker_0";
        this->m_line.header.stamp = this->text.header.stamp = ros::Time::now();
        this->m_line.ns = this->text.ns = "marker_node";
        this->m_line.action = this->text.action = visualization_msgs::Marker::ADD;
        this->m_line.pose.orientation.w = this->text.pose.orientation.w = 1.0;
        this->m_line.pose.orientation.x = this->text.pose.orientation.w = 0.0;
        this->m_line.pose.orientation.y = this->text.pose.orientation.w = 0.0;
        this->m_line.pose.orientation.z = this->text.pose.orientation.w = 0.0;

        this->m_line.id = this->text.id = 1;
        this->m_line.type = visualization_msgs::Marker::LINE_LIST;
        this->text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        this->m_line.scale.x = 0.01;
        this->m_line.scale.y = 0.01;
        this->m_line.scale.z = this->text.scale.z = 0.01;
        this->m_line.color.g = this->text.color.g =2.0;
        this->m_line.color.a = this->text.color.a =2.0;
        //this->m_line.lifetime = ros::Duration(1);

        
        string human_topic = "/HumanPose_3d_" + to_string(camera_id);
        human_sub = nh.subscribe(human_topic, 10, &Camera::humanPoseCallback, this);

        listener = new tf::TransformListener;
        br = new tf::TransformBroadcaster;

    }

    ~Camera(){
        delete listener;
        delete br;
        cout << "Release all pointer." << endl;
    }
    
    void calibrate();
    void humanPoseCallback(const human_pose_msgs::HumanList humanList);
    void triangulation();
    void addLine();
    void addLine(visualization_msgs::Marker& line_list, HumanPose::Pose& pose);
    void publish_3d_poses(vector<vector<HumanPose::Joint3D>>& pose_3d);
    void publish_3d_tf();
    
    void setFramePose();
private:
    int camera_id;
    ros::NodeHandle nh;
    ros::Subscriber human_sub;
    ros::Publisher *marker_pub;

    tf::TransformListener *listener;
    tf::TransformBroadcaster *br;
    
    //camera info
    Eigen::Matrix3d kinect_rot;
    Eigen::Matrix<double, 3, 1> kinect_trans;

    //all_pose info from marker
    vector<tf::Transform> TfPose;
    visualization_msgs::Marker m_line, text;
    visualization_msgs::MarkerArray markerarray;
};

void Camera::calibrate(){
    tf::StampedTransform transform;
	char kinect_id[50];
	sprintf(kinect_id,"camera_base_%d",this->camera_id);
    while(ros::ok()) {
        try
        {
            listener->waitForTransform("/marker_0",kinect_id,ros::Time(0),ros::Duration(3.0));
            listener->lookupTransform("/marker_0",kinect_id,ros::Time(0),transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        Eigen::Quaterniond q(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());
        Eigen::Vector3d trans(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
        this->kinect_rot=q.toRotationMatrix();
        this->kinect_trans = trans;
        break;
    }
	ROS_INFO("Camera_%d pose has listener sunccessfully!", this->camera_id);

    delete listener;
}

void Camera::humanPoseCallback(const human_pose_msgs::HumanList humanList)
{
    SourecePose.clear();
    int humanNum = humanList.human_list.size();
    if(humanNum > 0)
    {
        for(int person=0; person<humanNum; ++person)
        {
            auto body_keypoints = humanList.human_list[person].body_key_points_with_prob;
            int count = 0;
	        double prob_sum = 0.0;
	        for(int j=0;j < body_keypoints.size();j++)
	        {
	        	if(body_keypoints[j].prob > 0.0)
	            {
					prob_sum += body_keypoints[j].prob;
					count++;
				}
			}
			double prob_eval = prob_sum/count;
			if(prob_eval < 0.2)
			{
				continue;
			}
            HumanPose::Pose new_pose;
            new_pose.setLabel(-1);
            new_pose.setPose(humanList, person);
            SourecePose.push_back(new_pose);        
        }
    }
}

void Camera::triangulation(){
    if(SourecePose.size() == 0){
        ROS_INFO("No human");
        return;
    }

    TransPose.clear();
    TransPose.resize(SourecePose.size());
    for(int human_index=0; human_index < SourecePose.size(); ++human_index){
        auto human = SourecePose[human_index];
        vector<HumanPose::PointWithProb> pose_wor(human.pose.size());
        Eigen::Matrix<double, 3, 1> joint_point_cam, joint_point_wor;
        for(int joint_id = 0; joint_id < human.pose.size(); ++joint_id){
            if(human.pose[joint_id].p != -1 && human.pose[joint_id].id == joint_id){
                joint_point_cam(0, 0) = human.pose[joint_id].x / 100;
                joint_point_cam(1, 0) = human.pose[joint_id].y / 100;
                joint_point_cam(2, 0) = human.pose[joint_id].z / 100;
            }
            else
            {
                continue;
            }
            joint_point_wor = this->kinect_rot.cast<double>() * joint_point_cam + this->kinect_trans.cast<double>();
            
            //update
            TransPose[human_index].pose[joint_id].id = human.pose[joint_id].id;
            TransPose[human_index].pose[joint_id].p = human.pose[joint_id].p;
            TransPose[human_index].pose[joint_id].x = joint_point_wor(0, 0);
            TransPose[human_index].pose[joint_id].y = joint_point_wor(1, 0);
            TransPose[human_index].pose[joint_id].z = joint_point_wor(2, 0);
            TransPose[human_index].setLabel(-1);
            //SourecePose[human_index].setLabel(human_index);
        }
    }

    // for(int i=0; i<TransPose.size(); ++i){
    //     cout << "Camera_" << this->camera_id << "  Human_" << i << "  Label: " << TransPose[i].getLabel() << endl;
    //     for(int joint_id = 0; joint_id < TransPose[i].pose.size(); ++joint_id){
    //         cout << TransPose[i].pose[joint_id].id << " "
    //              << TransPose[i].pose[joint_id].x << " "
    //              << TransPose[i].pose[joint_id].y << " "
    //              << TransPose[i].pose[joint_id].z << " "
    //              << TransPose[i].pose[joint_id].p << endl;
    //     }
    // }
    // ROS_INFO("Camera_%d triangulation end.", this->camera_id);
}

void Camera::publish_3d_poses(vector<vector<HumanPose::Joint3D>>& pose_3d) {
    this->m_line.points.clear();
    for(int i = 0;i<pose_3d.size();i++)
    {
        for(int j = 0;j<pairNum;j++)
        {
            geometry_msgs::Point p1,p2;
            int k1 = bodyEages[j][0];
            int k2 = bodyEages[j][1];

            p1.x = pose_3d[i][k1].x;
            p1.y = pose_3d[i][k1].y;
            p1.z = pose_3d[i][k1].z;
            this->m_line.points.push_back(p1); 

            p2.x = pose_3d[i][k2].x;      
            p2.y = pose_3d[i][k2].y;
            p2.z = pose_3d[i][k2].z;
            this->m_line.points.push_back(p2);
        }
        // geometry_msgs::Pose pose;
        // pose.position.x = (float)WorldPose[i][0](0, 0);
        // pose.position.y = (float)WorldPose[i][0](1, 0);
        // pose.position.z = (float)WorldPose[i][0](2, 0);
        // this->text.pose = pose;
        // this->text.text = "id= " + to_string(i);
    }
    
    // this->markerarray.markers.push_back(this->m_line);
    // this->markerarray.markers.push_back(this->text);
    marker_pub->publish(this->m_line);
    
    // // marker_pub->publish(this->text);
    // marker_pub->publish(this->markerarray);
}


/*
void Camera::publish_3d_tf(){
    TfPose.clear();

    //set
    for(int i=0; i<WorldPose.size(); ++i){
        //HumanPose::Pose pose = WorldPose[i];
        tf::Transform joint(tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(WorldPose[i][11].x, WorldPose[i][11].y, WorldPose[i][11].z));

        TfPose.push_back(joint);   
    }

    //pub
    for(int i=0; i<TfPose.size(); ++i){
        ostringstream oss;
        oss << "human_" << i << "/rHand";
        br->sendTransform(tf::StampedTransform(TfPose[i], ros::Time::now(), "marker_0", oss.str()));
    }
    
}
*/

int main(int argc,char **argv)
{
    ros::init(argc,argv,"human_pose");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("human_pose",10);

    // //创建一个char型变量是为了记录不同的模式
    // unsigned char mode = '0';

    if(argc <= 1){
        cout << "Please choose one camera or multiple cameras!\n" << "cam_a or cam_a and cam_b" << endl;
        ros::shutdown();//退出
    }
    // else if(argc > 1){
    //     Camera *cam_a = new Camera(1, pub);
    //     cam_a->calibrate();
    //     mode = '1';
    // }
    // else if(argc > 2){
    //     Camera cam_a(1, pub);
    //     Camera cam_b(2, pub);
    //     cam_a.calibrate();
    //     cam_b.calibrate();    

    //     mode = '2';
    // }

    Camera *cam_a = new Camera(1, pub);
    cam_a->calibrate();

    HumanPose::Associate assoc;
    HumanPose::Track track;

    while(ros::ok())
    { 
        vector<vector<HumanPose::Joint3D>> pose_3d = cam_a->triangulation(); //转换到世界坐标系下

        //tracking
        track.addFramePose()        
        

        //cam_b.triangulation();
        //assoc.generatePair(cam_a.TransPose, cam_b.TransPose);
        //assoc.associate();
        
        //vector<vector<HumanPose::Joint3D>> pose_3d = assoc.extract3DPoses();
        // track.addFramePose(pose_3d);
        //track.testNRandQueue();

        //cam_a.publish_3d_tf();
        //cam_a.publish_3d_poses(pose_3d);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
