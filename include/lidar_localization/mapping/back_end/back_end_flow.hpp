//
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_BACK_END_FLOW_HPP_
#define _LIDAR_LOCALIZATION_BACK_END_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include "lidar_localization/subscriber/loop_pose_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/key_frame_publisher.hpp"
#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include "lidar_localization/mapping/back_end/back_end.hpp"


namespace lidar_localization
{

class BackEndFlow
{
public:
    BackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

    bool Run();

    bool ForceOptimize();

private:
    bool ReadData();                // 读数据
    bool MaybeInsertLoopPose();
    bool HasData();
    bool ValidData();
    bool UpdateBackEnd();
    bool PublishData();


private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;


};

}


#endif // _LIDAR_LOCALIZATION_BACK_END_FLOW_HPP_
