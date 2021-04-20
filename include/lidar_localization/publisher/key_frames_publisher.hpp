//
// For back_end_flow.hpp
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_KEY_FRAMES_PUBLISHER_HPP_
#define _LIDAR_LOCALIZATION_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_localization/sensor_data/key_frame.hpp"


namespace lidar_localization
{

class KeyFramesPublisher
{
public:
    KeyFramesPublisher(ros::NodeHandle& nh,
                       std::string topic_name,
                       std::string frame_id,
                       int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frames);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";

};

}


#endif // _LIDAR_LOCALIZATION_KEY_FRAMES_PUBLISHER_HPP_
