/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization
{

NDTRegistration::NDTRegistration(const YAML::Node& node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
{
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
{
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}


bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
{
    ndt_ptr_->setResolution(res);                       // NDT网格结构的分辨率  1.0
    ndt_ptr_->setStepSize(step_size);                   // 为More-Thuente线搜索设置最大步长  0.1
    ndt_ptr_->setTransformationEpsilon(trans_eps);      // 为终止条件设置最大转换差异  0.01
    ndt_ptr_->setMaximumIterations(max_iter);           // 匹配迭代最大次数  30

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: "  << step_size << ", "
              << "trans_eps: "  << trans_eps << ", "
              << "max_iter: "   << max_iter
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target)
{
    ndt_ptr_->setInputTarget(input_target);

    return true;
}


bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose)
{
    ndt_ptr_->setInputSource(input_source);             // 要匹配的点云
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);      // 点云配准
    // output_cloud: 存储input_source经过配准后的点云(由于input_source被极大的降采样了,因此这个数据没什么用)

    result_pose = ndt_ptr_->getFinalTransformation();         // 获得优化后的位姿

    return true;
}

float NDTRegistration::GetFitnessScore()
{
    return ndt_ptr_->getFitnessScore();
}

}