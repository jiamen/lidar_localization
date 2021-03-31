//
// Created by zlc on 2021/3/28.
//

#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"       // 打印


// 静态成员变量必须在类外初始化
double lidar_localization::GNSSData::origin_longitude = 0.0;
double lidar_localization::GNSSData::origin_latitude = 0.0;
double lidar_localization::GNSSData::origin_altitude = 0.0;
bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;


namespace lidar_localization
{

void GNSSData::InitOriginPosition()
{
    geo_converter.Reset(latitude, longitude, altitude);

    origin_longitude = longitude;       // 经度
    origin_latitude  = latitude;        // 纬度
    origin_altitude  = altitude;        // 海拔

    origin_position_inited = true;
}

void GNSSData::UpdateXYZ()
{
    if (!origin_position_inited)
    {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }

    geo_converter.Forward(latitude,longitude,altitude,local_E,local_N,local_U);
}


// 得到同步数据
bool GNSSData::SyncData(std::deque<GNSSData> &UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time)
{
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2)
    {
        // 如果开始的时间就比同步时间大，说明整体已经偏后，不可能有同步时间sync左右的时间数据，直接返回错误
        if (UnsyncedData.front().time > sync_time)
            return false;
        // 直接查看第2个数据，第2个数据小于同步时间，第1个肯定小于同步时间，直接将第1个弹出
        if (UnsyncedData.at(1).time < sync_time)
        {
            UnsyncedData.pop_front();
            continue;
        }
        // 时间离得太远直接删掉
        if (sync_time - UnsyncedData.front().time > 0.2)
        {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2)
        {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    // 经过循环会返回UnsyncedData前两个数据一大一小在sync_time之间
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;


    double front_scale = (back_data.time - sync_time)  / (back_data.time - front_data.time);
    double back_scale  = (sync_time - front_data.time) / (back_data.time - front_data.time);

    // 得到最终的同步数据
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    // 经纬海拔
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude  = front_data.latitude  * front_scale + back_data.latitude  * back_scale;
    synced_data.altitude  = front_data.altitude  * front_scale + back_data.altitude  * back_scale;
    // 东北天
    synced_data.local_E   = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N   = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U   = front_data.local_U * front_scale + back_data.local_U * back_scale;


    SyncedData.push_back(synced_data);

    return true;
}


}




