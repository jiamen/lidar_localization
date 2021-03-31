//
// Created by zlc on 2021/3/29.
//

#include "lidar_localization/sensor_data/velocity_data.hpp"

#include "glog/logging.h"

namespace lidar_localization
{

bool VelocityData::SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData,
                            double sync_time)
{
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2)
    {
        if ( UnsyncedData.front().time > sync_time )
            return false;
        if ( UnsyncedData.at(1).time < sync_time )
        {
            UnsyncedData.pop_front();
            continue;
        }

        // 隔得太久直接跳出
        if ( sync_time - UnsyncedData.front().time > 0.2 )
        {
            UnsyncedData.pop_front();
            break;
        }
        if ( UnsyncedData.at(1).time - sync_time > 0.2 )
        {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    // 没有找到满足同步时间附近要求的左右两个数据，直接返回false
    if ( UnsyncedData.size() < 2 )
        return false;

    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    VelocityData synced_data;

    double front_scale = (back_data.time - sync_time)  / (back_data.time - front_data.time);
    double back_scale  = (sync_time - front_data.time) / (back_data.time - front_data.time);

    synced_data.time = sync_time;
    synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}



// 激光雷达扫描过程中采集的数据中提供的速度是IMU所处位置的速度，我们需要将其转化到激光雷达所处位置的速度（线速度+角速度）
// imu转雷达坐标系：线速度、角速度
void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix)
{
    // 杆臂矩阵
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();           // 哪个坐标系转到哪个坐标系
    Eigen::Matrix3d t_R = matrix.block<3,3>(0,0);

    /*
     * 更新公式：
     *  w_1 = R * w0
     *  v_1 = R * v0 + △v = R*v0 + w1^t
     *                | 0  -w2  w1 |
     *  △v = w1 × t = | w2  0  -w0 |
     *                |-w1  w0  0  |
     * */

    // 角速度更新
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    w = t_R * w;
    // 线速度先旋转
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    v = t_R * v;

    // 这部分坐标如何进行转换？
    // 再加上叉乘结果，杆臂速度 = 角速度 叉乘 杆臂
    Eigen::Vector3d r(matrix(0,3), matrix(1,3), matrix(2,3));
    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * r(2) - w(2) * r(1);
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(1) * r(1) - w(1) * r(0);
    v = v + delta_v;

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}


}


