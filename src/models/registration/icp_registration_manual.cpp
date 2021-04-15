//
// Created by zlc on 2021/4/10.
//

#include "lidar_localization/models/registration/icp_registration_manual.hpp"
#include "glog/logging.h"
#include <Eigen/Dense>


namespace lidar_localization
{

ICPRegistrationManual::ICPRegistrationManual(const YAML::Node &node)
        : kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>) {
    float max_correspond_dis = node["max_correspondence_distance"].as<float>();
    int max_iter = node["max_iter"].as<int>();      //最大迭代次数
    SetRegistrationParam(max_correspond_dis, max_iter);
}

ICPRegistrationManual::ICPRegistrationManual(float max_correspond_dis, int max_iter)
        : kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>) {
    SetRegistrationParam(max_correspond_dis, max_iter);
}

bool ICPRegistrationManual::SetRegistrationParam(float max_correspond_dis, int max_iter) {
    max_correspond_distance_ = max_correspond_dis;
    max_iterator_ = max_iter;

    LOG(INFO) << "ICP Manual 的匹配参数为: " << std::endl
              << "max_correspond_dis: " << max_correspond_dis << ", "
              << "max_iter: " << max_iter << std::endl
              << std::endl;

    return true;
}


bool ICPRegistrationManual::SetInputTarget(
        const CloudData::CLOUD_PTR &input_target) {
    target_cloud_.reset(new CloudData::CLOUD);
    target_cloud_ = input_target;
    kdtree_ptr_->setInputCloud(input_target);
    return true;
}


bool ICPRegistrationManual::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                      const Eigen::Matrix4f &predict_pose,
                                      CloudData::CLOUD_PTR &result_cloud_ptr,
                                      Eigen::Matrix4f &result_pose)
{
    transformation_ = predict_pose;                         // T_rc
    rotation_matrix_ = transformation_.block<3, 3>(0, 0);       // 取旋转矩阵
    translation_ = transformation_.block<3, 1>(0, 3);              // 取平移矩阵

    calculateTrans(input_source);       // 计算变换矩阵,经过这个计算之后transformation_会发生改变

    pcl::transformPointCloud(*input_source, *result_cloud_ptr, transformation_);   // 对点云进行变换
    result_pose = transformation_;

    return true;
}


void ICPRegistrationManual::calculateTrans(const CloudData::CLOUD_PTR &input_cloud)
{
    CloudData::CLOUD_PTR transformed_cloud(new CloudData::CLOUD);
    int knn = 1;    // 进行1nn的搜索
    int iterator_num = 0;

    while (iterator_num < max_iterator_)
    {
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, transformation_);    // 先按照初次传入的不精准变换矩阵的对点云进行变换
        Eigen::Matrix<float, 6, 6> Hessian;
        Eigen::Matrix<float, 6, 1> B;
        Hessian.setZero();
        B.setZero();

        for (size_t i = 0; i < transformed_cloud->size(); ++i) {
            auto ori_point = input_cloud->at(i);
            if (!pcl::isFinite(ori_point))      // 判断原点云取出的点是否为有限数，原点云无效，则跳过不处理
                continue;

            auto transformed_point = transformed_cloud->at(i);
            std::vector<float> distances;
            std::vector<int> indexes;
            // 这里对点在kdtree组织的target_cloud中进行最近邻搜索，实际只搜索一个点，也就是找到最近点
            kdtree_ptr_->nearestKSearch(transformed_point, knn, indexes, distances);    // knn搜索
            if (distances[0] > max_correspond_distance_)    // 最近点大于距离阈值
            {
                continue;
            }

            // 从目标点云中取出最近点
            Eigen::Vector3f closest_point = Eigen::Vector3f(target_cloud_->at(indexes[0]).x,
                                                            target_cloud_->at(indexes[0]).y,
                                                            target_cloud_->at(indexes[0]).z);



            // 构建残差f：计算初次变换矩阵点 与 目标点云中最邻近点 的距离
            Eigen::Vector3f err_dis =
                    Eigen::Vector3f(transformed_point.x, transformed_point.y, transformed_point.z) - closest_point;

            Eigen::Matrix<float, 3, 6> Jacobian(Eigen::Matrix<float, 3, 6>::Zero());
            // 残差对平移的导数，放在前三列
            Jacobian.leftCols<3>() = Eigen::Matrix3f::Identity();
            // 残差对旋转的导数，放在后三列，这里使用右导数
            Jacobian.rightCols<3>() =
                    -rotation_matrix_ * Sophus::SO3f::hat(Eigen::Vector3f(ori_point.x, ori_point.y, ori_point.z));

            Hessian += Jacobian.transpose() * Jacobian;
            B += -Jacobian.transpose() * err_dis;
        }

        iterator_num++;                    // 迭代次数+1
        if (0 == Hessian.determinant())     // Hessian矩阵的行列式为1
        {
            continue;
        }
        Eigen::Matrix<float, 6, 1> delta_x = Hessian.inverse() * B;

        // x = x + △x
        translation_ += delta_x.head<3>();
        auto delta_rotation = Sophus::SO3f::exp(delta_x.tail<3>());
        rotation_matrix_ *= delta_rotation.matrix();

        transformation_.block<3, 3>(0, 0) = rotation_matrix_;
        transformation_.block<3, 1>(0, 3) = translation_;
    }

}

}
