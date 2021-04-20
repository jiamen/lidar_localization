//
// Created by zlc on 2021/4/19.
//

#ifndef _LIDAR_LOCALIZATION_KEY_FRAME_HPP_
#define _LIDAR_LOCALIZATION_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lidar_localization
{

class KeyFrame
{
public:
    double time = 0.0;
    unsigned int   index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

public:
    Eigen::Quaternionf GetQuaternion();
};

}


#endif // _LIDAR_LOCALIZATION_KEY_FRAME_HPP_
