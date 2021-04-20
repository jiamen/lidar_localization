//
// Created by zlc on 2021/4/20.
//

#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include <Eigen/Dense>

namespace lidar_localization
{

KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id,
                                       int buff_size) : nh_(nh), frame_id_(frame_id)
{
    // 向ros Master端完成发布消息注册
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

// 数据发布，将自定义key_frame 转化为 ros消息发送出去
void KeyFramesPublisher::Publish(const std::deque<KeyFrame>& key_frames)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;           // 坐标系ID

    for ( size_t i=0; i<key_frames.size(); i ++ )
    {
        KeyFrame key_frame = key_frames.at(i);

        geometry_msgs::PoseStamped pose_stamped;
        ros::Time ros_time((float)key_frame.time);
        pose_stamped.header.stamp = ros_time;
        pose_stamped.header.frame_id = frame_id_;       // 坐标系Id

        pose_stamped.header.seq = key_frame.index;      // 关键帧索引

        pose_stamped.pose.position.x = key_frame.pose(0,3);
        pose_stamped.pose.position.y = key_frame.pose(1,3);
        pose_stamped.pose.position.z = key_frame.pose(2,3);

        Eigen::Quaternionf q = key_frame.GetQuaternion();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    publisher_.publish(path);
}

bool KeyFramesPublisher::HasSubscribers()
{
    return publisher_.getNumSubscribers() != 0;
}

}

