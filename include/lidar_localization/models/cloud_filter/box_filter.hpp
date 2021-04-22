//
// Created by zlc on 2021/4/21.
//

#ifndef _LIDAR_LOCALIZATION_BOX_FILTER_HPP_
#define _LIDAR_LOCALIZATION_BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization
{

class BoxFilter : public CloudFilterInterface
{
public:
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filter_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

private:
    void CalculateEdge();

private:
    pcl::CropBox<CloudData::POINT> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;

};

}


#endif // _LIDAR_LOCALIZATION_BOX_FILTER_HPP_
