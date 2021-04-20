//
// For back_end_flow.hpp
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_KEY_FRAME_PUBLISHER_HPP_
#define _LIDAR_LOCALIZATION_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
// 该消息表示带有时间标签和参考坐标的估计位姿。具体见解析
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// 将自定义数据转换为ros数据
#include "lidar_localization/sensor_data/key_frame.hpp"


namespace lidar_localization
{

class KeyFramePublisher
{
public:
    KeyFramePublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string frame_id,
                      int buff_size);
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame);

    // 本发布类是否有订阅者
    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

    std::string frame_id_ = "";
};

}


#endif // _LIDAR_LOCALIZATION_KEY_FRAME_PUBLISHER_HPP_
