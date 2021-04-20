//
// Created by zlc on 2021/4/20.
//

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization
{

Eigen::Quaternionf KeyFrame::GetQuaternion()
{
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}

}

