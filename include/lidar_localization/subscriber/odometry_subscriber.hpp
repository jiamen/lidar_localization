//
// For back_end_flow.hpp
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_ODOMETRY_SUBSCRIBER_HPP_
#define _LIDAR_LOCALIZATION_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization
{

class OdometrySubscriber
{
public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData>& deque_pose_data);

private:
    // 将ros类型的消息nav_OdometryConstPtr转换为自己写的数据类型
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PoseData> new_pose_data_;

    std::mutex buff_mutex_;     // 线程锁
};

}

#endif //LIDAR_LOCALIZATION_ODOMETRY_SUBSCRIBER_HPP
