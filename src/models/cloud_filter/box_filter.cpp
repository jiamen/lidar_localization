// @Description: 从点云中截取一个立方体部分
// Created by zlc on 2021/4/21.
//

#include <vector>
#include <iostream>
#include "glog/logging.h"

#include "lidar_localization/models/cloud_filter/box_filter.hpp"

namespace lidar_localization
{

BoxFilter::BoxFilter(YAML::Node node)
{
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3);

    for (size_t i=0; i < size_.size(); i ++)
    {
        size_.at(i) = node["box_filter_size"][i].as<float>();
    }

    SetSize(size_);
}

bool BoxFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                       CloudData::CLOUD_PTR& filter_cloud_ptr)
{
    filter_cloud_ptr->clear();

    pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
    pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
    pcl_box_filter_.setInputCloud(input_cloud_ptr);
    pcl_box_filter_.filter(*filter_cloud_ptr);

    return true;
}

void BoxFilter::SetSize(std::vector<float> size)
{
    size_ = size;
    std::cout << "Box Filter 的尺寸为: " << std::endl
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
              << std::endl << std::endl;

    CalculateEdge();
}

// 设置原点
void BoxFilter::SetOrigin(std::vector<float> origin)
{
    origin_ = origin;
    CalculateEdge();
}

// 计算边
void BoxFilter::CalculateEdge()
{
    for (size_t i = 0; i < origin_.size(); ++i)
    {
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
    }
}

// 得到边
std::vector<float> BoxFilter::GetEdge()
{
    return edge_;
}


}





