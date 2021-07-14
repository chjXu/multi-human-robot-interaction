#pragma once

#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


class Grapper{
public:
    Grapper(){}
    ~Grapper(){}

    void listener_tf();
    void setParamters();
    bool intersectionLinePlane(Eigen::Vector3f &p1, Eigen::Vector3f p2, const Eigen::Vector4f &plane, Eigen::Vector3f &crossP);
    void grapper();
private:
    Eigen::Vector3f cross_point; //待求解的点
    Eigen::Vector4f plane; //平面
    Eigen::Vector3f point_1;
    Eigen::Vector3f point_2;

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster br;
    tf::StampedTransform rWrist;
    tf::StampedTransform rElbow;
};