//
// 存放处理后的IMU姿态以及GNSS位置
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_POSE_DATA_HPP_
#define _LIDAR_LOCALIZATION_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization
{

class PoseData
{
public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

public:
    Eigen::Quaternionf GetQuaternion();
};

}

#endif //LIDAR_LOCALIZATION_POSE_DATA_HPP
