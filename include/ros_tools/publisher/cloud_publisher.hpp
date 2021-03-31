//
// Created by zlc on 2021/3/29.
//

#ifndef _LIDAR_LOCALIZATION_CLOUD_PUBLISHER_HPP_
#define _LIDAR_LOCALIZATION_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>    // 变换库


#include "lidar_localization/sensor_data/cloud_data.hpp"


namespace lidar_localization
{

class CloudPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher  publisher_;
    std::string     frame_id_;

private:
    void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);

public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    CloudPublisher() = default;

    void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);
    void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);

    bool HasSubscribers();

};

}



#endif // _LIDAR_LOCALIZATION_CLOUD_PUBLISHER_HPP_
