/*
 * @Description: GNSS数据定义
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef _LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define _LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>   // 由于ros的缓冲区机制，这里使用容器来存放gps传感器数据而不用单个变量
// 用于执行地理、UTM、UPS、MGRS、地心和局部笛卡尔坐标之间的转换，用于重力（例如EGM2008）、大地水准面高度和地磁场（例如，WMM2020）计算，以及用于解决测地线问题。
#include "Geocentric/LocalCartesian.hpp"

namespace lidar_localization
{

class GNSSData
{
public:
    double time = 0.0;          // 数据到达时间
    double longitude= 0.0;      // 经度
    double latitude = 0.0;      // 纬度
    double altitude = 0.0;      // 海拔
    double local_E = 0.0;       // 局部坐标的东北天
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;     // 原点经度
    static double origin_latitude;      // 原点纬度
    static double origin_altitude;      // 原点海拔高度

private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;     // 初始化标志，如果初始化则置1

public:
    void InitOriginPosition();
    void UpdateXYZ();
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};

}
#endif