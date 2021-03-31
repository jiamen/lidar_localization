//
// Created by zlc on 2021/3/29.
//

#include "ros_tools/subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization
{

GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
{
    buff_mutex_.lock();

    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude  = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude  = nav_sat_fix_ptr->altitude;
    gnss_data.status    = nav_sat_fix_ptr->status.status;
    gnss_data.services  = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);

    buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSData> &deque_gnss_data)
{
    buff_mutex_.lock();

    if (new_gnss_data_.size() > 0)
    {
        deque_gnss_data.insert(deque_gnss_data.end(),new_gnss_data_.begin(),new_gnss_data_.end());
        new_gnss_data_.clear();
    }

    buff_mutex_.unlock();
}

}


