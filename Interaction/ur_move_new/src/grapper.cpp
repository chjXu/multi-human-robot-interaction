#include <ur_move_new/grapper.h>
void Grapper::listener_tf(){
    while(ros::ok()){
        try
        {
            tf_listener.waitForTransform("base", "human_0/rElbow", ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("base", "human_0/rElbow", ros::Time(0), rElbow);
            tf_listener.waitForTransform("base", "human_0/rWrist", ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("base", "human_0/rWrist", ros::Time(0), rWrist);
            break;
        }
        catch(tf::TransformException ex)
        {
            continue;
        }
    }
}

void Grapper::setParamters(){
    //设置两个点
    point_1 = Eigen::Vector3f(rWrist.getOrigin().x(), rWrist.getOrigin().y(), rWrist.getOrigin().z());
    point_2 = Eigen::Vector3f(rElbow.getOrigin().x(), rElbow.getOrigin().y(), rElbow.getOrigin().z());
    plane = Eigen::Vector4f(0,0,1,0);
}

bool Grapper::intersectionLinePlane(Eigen::Vector3f &p1, Eigen::Vector3f p2, const Eigen::Vector4f &plane, Eigen::Vector3f &crossP)
{
    //分子
    auto P1D = plane[0] * p1[0] + plane[1] * p1[1] + plane[2] * p1[2];
    P1D = std::abs(P1D);
    auto P2D = plane[0] * p2[0] + plane[1] * p2[1] + plane[2] * p2[2];
    P2D = std::abs(P2D);

    if(P1D < P2D){
        std::swap(p1, p2);
        P1D = P2D;
    }

    const Eigen::Vector3f P1P2 = p2 - p1;

    //分母
    auto P1D2 = plane[0] * P1P2[0] + plane[1] * P1P2[1] + plane[2] * P1P2[2];
    P1D2 = std::abs(P1D2);
    if(P1D2 < FLT_EPSILON)
        return false;

    auto m = P1D / P1D2;
    crossP = p1 + m*P1P2;
    return true;
}

void Grapper::grapper(){
    while(ros::ok()){
    listener_tf();
    setParamters();
    if(intersectionLinePlane(point_1, point_2, plane, cross_point)){
        tf::Transform desired_goal;
        desired_goal.setOrigin(tf::Vector3(cross_point[0], cross_point[1], cross_point[2]+0.2));
        desired_goal.setRotation(tf::Quaternion(0.375, 0.925, -0.011, 0.062));
        br.sendTransform(tf::StampedTransform(desired_goal, ros::Time::now(), "base", "desired_goal"));
    }
    }
}