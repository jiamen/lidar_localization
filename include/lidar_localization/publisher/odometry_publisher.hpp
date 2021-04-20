/*
 * @Description: odometry 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef _LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define _LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
// 导航功能包要求机器人 能够通过里程计信息源发布包含速度信息的里程计nav_msgs/Odometry 消息；
#include <nav_msgs/Odometry.h>

namespace lidar_localization
{

class OdometryPublisher
{
public:
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix);

    bool HasSubscribers();

private:
    void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;       // 将要发布的ros里程计消息
};

}

#endif