//
// Created by zlc on 2021/4/20.
//

#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization
{
                                        // 节点句柄                    话题                缓存区大小
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
                                        // 使用自己写的数据回调函数对订阅得到的ros里程计数据进行处理
}

// 回调函数：将订阅的ros数据类型转换为自写的传感器数据类型PoseData
void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr)
{
    buff_mutex_.lock();
    PoseData pose_data;
    pose_data.time = odom_msg_ptr->header.stamp.toSec();

    // set the position  平移部分
    pose_data.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
    pose_data.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
    pose_data.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

    // ros中得到数据的旋转部分
    Eigen::Quaternionf q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;
    pose_data.pose.block<3,3>(0,0) = q.matrix();    // 四元数转换为变换矩阵的旋转矩阵部分

    new_pose_data_.push_back(pose_data);
    buff_mutex_.unlock();
}

void OdometrySubscriber::ParseData(std::deque<PoseData>& deque_pose_data)
{
    buff_mutex_.lock();         // 加锁
    if ( new_pose_data_.size() > 0 )
    {
        deque_pose_data.insert(deque_pose_data.end(), new_pose_data_.begin(), new_pose_data_.end());
        new_pose_data_.clear();
    }
    buff_mutex_.unlock();       // 解锁
}

}

