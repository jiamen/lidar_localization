//
// Created by zlc on 2021/4/10.
//

#ifndef _LIDAR_LOCALIZATION_ICP_REGISTRATION_MANUAL_HPP_
#define _LIDAR_LOCALIZATION_ICP_REGISTRATION_MANUAL_HPP_

#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "lidar_localization/models/registration/registration_interface.hpp"    // 配准接口

#include "se3.hpp"            // 添加 se3


namespace lidar_localization
{

class ICPRegistrationManual : public RegistrationInterface
{
public:
    ICPRegistrationManual(const YAML::Node& node);
    ICPRegistrationManual(float max_correspond_dis, int max_iter);

    // 1. 输入目标点云：目标点云用来构建一个kdtree，方便待匹配的点云寻找最近点
    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;

    // 2. 输入待匹配的点云以及以及初始位姿R，t
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;

private:
    bool SetRegistrationParam(float max_correspond_dis, int max_iter);
    void calculateTrans(const CloudData::CLOUD_PTR& input_cloud);           // 计算变换矩阵

private:
    CloudData::CLOUD_PTR target_cloud_;
    pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_ptr_;

    float max_correspond_distance_;     // 阈值
    int max_iterator_;                  // 最大迭代次数


    Eigen::Matrix3f rotation_matrix_;   // 旋转矩阵
    Eigen::Vector3f translation_;       // 平移矩阵
    Eigen::Matrix4f transformation_;    // 接受初始的变换矩阵

};

}




#endif // _LIDAR_LOCALIZATION_ICP_REGISTRATION_MANUAL_HPP_
