//
// For back_end_flow.hpp
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_LOOP_POSE_SUBSCRIBER_HPP_
#define _LIDAR_LOCALIZATION_LOOP_POSE_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
// 该消息表示带有时间标签和参考坐标的估计位姿。具体见解析
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lidar_localization/sensor_data/loop_pose.hpp"


namespace lidar_localization
{

class LoopPoseSubscriber
{
public:
    LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    LoopPoseSubscriber() = default;
    void ParseData(std::deque<LoopPose>& loop_pose_buff);

private:
    // 带有不确定性的空间位姿
    void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<LoopPose> new_loop_pose_;

    std::mutex buff_mutex_;     // 线程锁
};

}


#endif //LIDAR_LOCALIZATION_LOOP_POSE_SUBSCRIBER_HPP
