//
// Created by zlc on 2021/4/9.
//

#include "lidar_localization/models/registration/icp_registration.hpp"

#include "glog/logging.h"


namespace lidar_localization
{

ICPRegistration::ICPRegistration(const YAML::Node &node)
    : icp_ptr_( new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>() )
{
    float max_dist = node["max_dist"].as<float>();              // 1.0
    float trans_eps = node["trans_eps"].as<float>();            // 0.01
    float eculi_eps = node["eculi_eps"].as<float>();            // 0.01
    int max_iter = node["max_iter"].as<int>();                  // 30

    SetRegistrationParam(max_dist, trans_eps, eculi_eps, max_iter);
}


ICPRegistration::ICPRegistration(float max_dist, float trans_eps, float eculi_eps, int max_iter)
    : icp_ptr_( new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>() )
{
    SetRegistrationParam(max_dist, trans_eps, eculi_eps, max_iter);
}


bool ICPRegistration::SetRegistrationParam(float max_dist, float trans_eps, float eculi_eps, int max_iter)
{
    icp_ptr_->setMaxCorrespondenceDistance(max_dist);
    icp_ptr_->setTransformationEpsilon(trans_eps);
    icp_ptr_->setEuclideanFitnessEpsilon(eculi_eps);
    icp_ptr_->setMaximumIterations(max_iter);

    LOG(INFO) << "ICP 的匹配参数为: " << std::endl
              << "max_dist: "  << max_dist  << ", "
              << "trans_eps: " << trans_eps << ", "
              << "eculi_eps: " << eculi_eps << ", "
              << "max_iter: "  << max_iter
              << std::endl << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target)
{
    icp_ptr_->setInputTarget(input_target);

    return true;
}


bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose)
{
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);

    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}

// pcl中的score定义是两份点云之间的误差（error），score越小表示匹配的越准；   √
// ndt论文对score的定义是变换后点云的在相应网格的概率，概率越大表示越准，score越大。
float ICPRegistration::GetFitnessScore()
{
    return icp_ptr_->getFitnessScore();
}

}

