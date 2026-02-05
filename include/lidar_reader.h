#ifndef LIDAR_READER_H
#define LIDAR_READER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#if defined(__has_include)
#  if __has_include(<pcl/io/velodyne_driver.h>)
#    include <pcl/io/velodyne_driver.h>
#  endif
#endif
#include <string>
#include <mutex>
#include <thread>
#include <memory>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class LidarReader {
public:
    LidarReader(std::string ip = "192.168.1.201", int port = 2368);
    ~LidarReader();
    
    bool initialize();
    bool start();
    void stop();
    
    PointCloud::Ptr getLatestCloud();
    bool hasNewData();
    uint64_t getTimestamp();
    
private:
    void readThread();
    
    std::string lidar_ip_;
    int lidar_port_;
    
    PointCloud::Ptr latest_cloud_;
    uint64_t latest_timestamp_;
    
    std::mutex cloud_mutex_;
    std::unique_ptr<std::thread> read_thread_;
    
    bool is_running_;
    bool has_new_data_;
};

#endif // LIDAR_READER_H
