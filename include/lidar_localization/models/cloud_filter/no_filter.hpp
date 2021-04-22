// @Description: 不滤波
// Created by zlc on 2021/4/21.
//

#ifndef _LIDAR_LOCALIZATION_NO_FILTER_HPP_
#define _LIDAR_LOCALIZATION_NO_FILTER_HPP_

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization
{

class NoFilter : public CloudFilterInterface
{
public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

};

}

#endif // _LIDAR_LOCALIZATION_NO_FILTER_HPP_
