/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef _LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define _LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace lidar_localization
{

class VelocityData
{
public:
    // 线速度
    struct LinearVelocity
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    // 角速度
    struct AngularVelocity
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;                      // velocity轮速计数据到达时间
    LinearVelocity  linear_velocity;        // 线速度
    AngularVelocity angular_velocity;       // 角速度
  
public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
};

}
#endif