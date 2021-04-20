/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization
{
// 里程计信息发布初始化
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
    : nh_(nh)
{

    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

// 自带时间发布
void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix, double time)
{
    ros::Time ros_time((float)time);
    PublishData(transform_matrix, ros_time);
}

// 这里的时间是现获取的
void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix)
{
    PublishData(transform_matrix, ros::Time::now());
}

// 私有函数，真正的发布里程计数据消息的函数部分
void OdometryPublisher::PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time)
{
    odometry_.header.stamp = ros::Time::now();

    // set the position  平移部分
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    // 旋转部分以四元数的形式发布
    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers()
{
    return publisher_.getNumSubscribers() != 0;
}

}
