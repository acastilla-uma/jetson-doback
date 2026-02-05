#ifndef CAMERA_READER_H
#define CAMERA_READER_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <thread>
#include <memory>

class CameraReader {
public:
    CameraReader(int device_id = 0);
    ~CameraReader();
    
    bool initialize();
    bool start();
    void stop();
    
    cv::Mat getRGB();
    cv::Mat getDepth();
    uint64_t getTimestamp();
    bool hasNewFrame();
    
private:
    void captureThread();
    
    int device_id_;
    
    rs2::pipeline pipeline_;
    rs2::config config_;
    
    cv::Mat latest_rgb_;
    cv::Mat latest_depth_;
    uint64_t latest_timestamp_;
    
    std::mutex frame_mutex_;
    std::unique_ptr<std::thread> capture_thread_;
    
    bool is_running_;
    bool has_new_frame_;
};

#endif // CAMERA_READER_H
