//
// Created by zlc on 2021/3/28.
//

#ifndef _LIDAR_LOCALIZATION_ODOMETRY_PUBLISHER_HPP_
#define _LIDAR_LOCALIZATION_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


namespace lidar_localization
{

class OdometryPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;

private:
    void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

public:
    OdometryPublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& tranform_matrix, double time);
    void Publish(const Eigen::Matrix4f& tranform_matrix);

    bool HasSubscribers();

};

}


#endif //LIDAR_LOCALIZATION_ODOMETRY_PUBLISHER_HPP
