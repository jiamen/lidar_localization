/*
 * @Description: 点云数据定义
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef _LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define _LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization
{

class CloudData
{
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;          // 定义简写的指针：pcl::PointCloud<pcl::PointXYZ>::Ptr

public:
    CloudData()
      : cloud_ptr(new CLOUD())
    {
    }

public:
    double time = 0.0;                      // 点云数据到达时间
    CLOUD_PTR cloud_ptr;
};

}

#endif