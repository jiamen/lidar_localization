//
// Created by zlc on 2021/4/15.
//

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace lidar_localization
{

// 获取载体运动信息，角速度angular_rate_、速度velocity_分别用来计算相对角度和相对位移
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data)
{
    scan_period_ = scan_period;                     // 扫描一帧周期，一帧是10Hz，也就是100ms，0.1s时间
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr)
{
    CloudData::CLOUD_PTR origin_cloud_ptr( new CloudData::CLOUD(*input_cloud_ptr) );
    output_cloud_ptr.reset( new CloudData::CLOUD() );

    float orientation_space = 2.0 * M_PI;           // 方向角一圈 2*pi
    float delete_space = 5.0 * M_PI / 180.0;        // 过滤正负5度激光点

    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);  // 起始点方位角（欧拉角） atan2(y/x)

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());     // 旋转向量，以Z轴为旋转轴，旋转方位角弧度
    Eigen::Matrix3f rotate_matrix = t_V.matrix();   // 旋转矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();         // 变换矩阵
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();

    // 原始点云旋转过后，保证成为新坐标系的方向
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);// 输入、输出、变换矩阵

    // 线速度、角速度
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    //
    for (size_t point_index=1; point_index<origin_cloud_ptr->points.size(); ++ point_index)
    {
        // 2.1 获得一帧点云每个点的方位角及预处理
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if ( orientation < 0.0 )        // 保证角度都是正的
            orientation += 2.0 * M_PI;

        // 过滤了±5°的点
        if ( orientation < delete_space || 2.0 * M_PI - orientation < delete_space )
            continue;

        // 2.2 根据顺序扫描时间对点云进行去畸变处理（旋转+平移）
        // 减去scan_period/2，把点云转换到该帧采集的中间时刻上去
        // bag里点云的时刻是该帧点云起始时刻和终止时刻的平均值
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_/2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);       // Ri = w△t
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;
        CloudData::POINT point;

        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);

        output_cloud_ptr->points.push_back(point);
    }

    // 2.3 点云去畸变后旋转
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());

    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time)
{
    // 角度=角速度*运行时间
    Eigen::Vector3f angle = angular_rate_ * real_time;

    // 旋转向量z-y-x
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());

    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;

    return t_V.matrix();        // 得到相对与T0的相对旋转矩阵Ti
}


}


