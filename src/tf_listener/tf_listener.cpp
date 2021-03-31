//
// Created by zlc on 2021/3/30.
//

#include "ros_tools/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>


namespace lidar_localization
{

TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id)
{

}

bool TFListener::LookupData(Eigen::Matrix4f &transform_matrix)
{
    try
    {
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
        TransFormToMatrix(transform, transform_matrix);

        return true;
    }
    catch (tf::TransformException& ex)
    {
        return false;
    }
}

bool TFListener::TransFormToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix)
{
    // child_frame_id 的原点
    Eigen::Translation3f tf_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    // 旋转矩阵
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw,   Eigen::Vector3f::UnitZ());

    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tf_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
}

}


