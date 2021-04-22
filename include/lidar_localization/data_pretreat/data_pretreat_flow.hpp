//
// Created by zlc on 2021/4/15.
// 数据预处理模块：包括时间同步、点云去畸变等
//


#ifndef _LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_
#define _LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>

// subscriber:订阅4种数据：雷达点云数据，雷达和点云的相对坐标系，GNSS组合导航信息、姿态、速度、角速度等
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"         // 组合导航：姿态、角速度    IMU
#include "lidar_localization/subscriber/velocity_subscriber.hpp"    // 组合导航：速度          轮速计
#include "lidar_localization/subscriber/gnss_subscriber.hpp"        // 组合导航：gnss姿态       GPS
#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher 输出：①畸变补偿后的点云  ②GNSS组合导航位置、姿态
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
// models 点云运动畸变补偿
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"


namespace lidar_localization
{

class DataPretreatFlow
{
public:
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();


private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber>   imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<GNSSSubscriber>  gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;              // 得到lidar到imu的变换矩阵

    // publisher 发布两段轨迹
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

    // models   点云补畸变
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;                 // 自定义点云数据队列
    std::deque<IMUData> imu_data_buff_;                     // 自定义imu数据队列          组合导航
    std::deque<VelocityData> velocity_data_buff_;           // 自定义轮速计速度数据队列    组合导航
    std::deque<GNSSData> gnss_data_buff_;                   // 自定义GPS导航数据队列      组合导航

    CloudData current_cloud_data_;
    IMUData   current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData  current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};

}



#endif // _LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_
