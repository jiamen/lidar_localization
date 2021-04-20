//
// Created by zlc on 2021/4/15.
//

#ifndef _LIDAR_LOCALIZATION_DISTORTION_ADJUST_HPP_
#define _LIDAR_LOCALIZATION_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"


namespace lidar_localization
{

class DistortionAdjust
{
public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

private:
    float scan_period_;                 // 扫描一帧的周期
    Eigen::Vector3f velocity_;          // 速度
    Eigen::Vector3f angular_rate_;      // 角速率
};

}



#endif // _LIDAR_LOCALIZATION_DISTORTION_ADJUST_HPP_
