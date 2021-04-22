//
// Created by zlc on 2021/4/15.
//

#ifndef LIDAR_LOCALIZATION_PRINT_INFO_HPP
#define LIDAR_LOCALIZATION_PRINT_INFO_HPP

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"


namespace lidar_localization
{

class PrintInfo
{
public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};

}

#endif //LIDAR_LOCALIZATION_PRINT_INFO_HPP
