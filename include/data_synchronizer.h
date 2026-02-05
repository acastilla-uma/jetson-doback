#ifndef DATA_SYNCHRONIZER_H
#define DATA_SYNCHRONIZER_H

#include "lidar_reader.h"
#include "camera_reader.h"
#include <string>
#include <memory>

struct SynchronizedData {
    PointCloud::Ptr point_cloud;
    cv::Mat rgb_image;
    cv::Mat depth_image;
    uint64_t lidar_timestamp;
    uint64_t camera_timestamp;
    int frame_id;
};

class DataSynchronizer {
public:
    DataSynchronizer(const std::string& output_dir, int64_t time_tolerance_ms = 50);
    
    bool initialize(std::shared_ptr<LidarReader> lidar, 
                   std::shared_ptr<CameraReader> camera);
    
    bool start();
    void stop();
    
    bool getSynchronizedData(SynchronizedData& out_data);
    bool processAndSave();
    
    int getTotalFramesSaved() const { return total_frames_saved_; }
    
private:
    bool synchronizeData(SynchronizedData& out_data);
    bool saveData(const SynchronizedData& data);
    void createOutputDirectories();
    
    std::string output_dir_;
    std::string pointcloud_dir_;
    std::string rgb_dir_;
    std::string depth_dir_;
    std::string metadata_file_;
    
    std::shared_ptr<LidarReader> lidar_reader_;
    std::shared_ptr<CameraReader> camera_reader_;
    
    int64_t time_tolerance_;
    int total_frames_saved_;
    int frame_counter_;
};

#endif // DATA_SYNCHRONIZER_H
