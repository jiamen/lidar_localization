//
// Created by zlc on 2021/4/15.
//

#ifndef _LIDAR_LOCALIZATION_TIC_TOC_HPP_
#define _LIDAR_LOCALIZATION_TIC_TOC_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>


namespace lidar_localization
{
class TicToc
{
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;

public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        start = std::chrono::system_clock::now();       // 重新开启计时
        return elapsed_seconds.count();
    }

};
}


#endif // _LIDAR_LOCALIZATION_TIC_TOC_HPP_
