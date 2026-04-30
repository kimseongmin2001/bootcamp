#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// 카메라 채널 하나를 담당 (sm_mjpeg_to_ros_node 패턴 참조)
class MjpegStream
{
public:
    MjpegStream(std::string url, std::string topic,
                rclcpp::Node* node, int w, int h);
    ~MjpegStream();

private:
    void   stream_loop();
    bool   extract_frame(std::vector<uint8_t>& buf, cv::Mat& out);
    static size_t curl_write_cb(char* ptr, size_t size, size_t nmemb, void* ud);

    std::string  url_;
    int          w_, h_;
    std::atomic<bool> running_{true};
    std::thread  thread_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Node* node_;
};

class MjpegNode : public rclcpp::Node
{
public:
    MjpegNode();
    ~MjpegNode();

private:
    std::unique_ptr<MjpegStream> front_;
    std::unique_ptr<MjpegStream> bottom_;
};
