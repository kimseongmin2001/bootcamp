#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

struct FlightState {
    double lat=0, lon=0, alt=0, pitch=0, roll=0, hdg=0, spd=0;
    bool   valid = false;
};

class VlmNode : public rclcpp::Node
{
public:
    VlmNode();
    ~VlmNode();

private:
    void front_cb(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void bottom_cb(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void state_cb(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

    void        infer_loop();
    void        publish_panel();
    std::string call_qwen(const cv::Mat& front, const cv::Mat& bottom,
                          const FlightState& f);
    cv::Mat     render_panel(const cv::Mat& front, const cv::Mat& bottom,
                             const std::string& text, const FlightState& f);

    static std::string encode_jpeg_b64(const cv::Mat& img, int quality);
    static std::string http_post(const std::string& url,
                                 const std::string& body, double timeout);

    std::string       qwen_url_;
    int               max_tokens_, jpeg_quality_;
    double            timeout_sec_;

    std::mutex        mutex_;
    cv::Mat           img_front_, img_bottom_;
    FlightState       flight_;
    std::string       last_response_{"-- Qwen 서버 대기 중 --"};

    std::atomic<bool> running_{true};
    std::thread       infer_thread_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_front_, sub_bottom_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr             pub_panel_;
    rclcpp::TimerBase::SharedPtr                                      panel_timer_;
};
