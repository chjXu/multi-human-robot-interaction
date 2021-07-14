#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_shape");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("Obstacles", 1);
    geometry_msgs::PointStamped myPoint;

    myPoint.point.x = 0.6;
    myPoint.point.y = 0.0;
    myPoint.point.z = 0.2;

    myPoint.header.frame_id = "base";

    ros::Rate rate(30);
    while (ros::ok())
    {
        pub.publish(myPoint);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}