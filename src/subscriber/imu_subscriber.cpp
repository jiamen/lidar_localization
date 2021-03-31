//
// Created by zlc on 2021/3/29.
//

#include "ros_tools/subscriber/imu_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization
{

// 构造函数里调用回调函数
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
{
    buff_mutex_.lock();

    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;

    new_imu_data_.push_back(imu_data);

    buff_mutex_.unlock();
}

// 解析参数
void IMUSubscriber::ParseData(std::deque<IMUData> &deque_imu_data)
{
    buff_mutex_.lock();

    if (new_imu_data_.size() > 0)
    {
        deque_imu_data.insert(deque_imu_data.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }

    buff_mutex_.unlock();
}

}
