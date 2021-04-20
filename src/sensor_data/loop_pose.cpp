//
// Created by zlc on 2021/4/20.
//

#include "lidar_localization/sensor_data/loop_pose.hpp"

namespace lidar_localization
{

Eigen::Quaternionf LoopPose::GetQuaternion()
{
    Eigen::Quaternionf q;

    q = pose.block<3,3>(0,0);   // 使用旋转矩阵直接给四元数赋值

    return q;
}

}

