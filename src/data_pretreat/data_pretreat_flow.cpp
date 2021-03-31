//
// Created by zlc on 2021/3/29.
//

#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"


namespace lidar_localization
{

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh, std::string cloud_topic)
{
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "velo_link");

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "velo/link", 100);
    gnss_pub_ptr_  = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "velo_link", 100);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run()
{
    if (!ReadData())
        return false;

    if (!InitCalibration())
        return false;

    while (HasData())
    {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData()
{
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    static std::deque<IMUData>  unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if ( 0 == cloud_data_buff_.size() )
        return false;



    return true;
}


bool DataPretreatFlow::InitCalibration()
{
    static bool calibration_received = false;
    if (!calibration_received)
    {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_))
        {
            calibration_received = true;
        }
    }

    return calibration_received;
}


bool DataPretreatFlow::InitGNSS()
{
    static bool gnss_inited = false;
    if ( !gnss_inited )
    {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData()
{
    if ( 0 == cloud_data_buff_.size() )
        return false;
    if ( 0 == imu_data_buff_.size() )
        return false;
    if ( 0 == velocity_data_buff_.size() )
        return false;
    if ( 0 == gnss_data_buff_.size() )
        return false;

    return true;
}

// 验证数据有效性
bool DataPretreatFlow::ValidData()
{
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_   = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time  = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if ( diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time< -0.05  )
    {
        cloud_data_buff_.pop_front();
        return false;
    }
    if ( diff_imu_time > 0.05 )
    {
        imu_data_buff_.pop_front();
        return false;
    }
    if ( diff_gnss_time > 0.05 )
    {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformDoubleData()
{
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;

    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData()
{
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);

    return true;
}


}


