#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <mutex>
#include <thread>

// 비행 상태 (패키지 내부 공통 구조체)
struct FlightState {
    double lat = 0, lon = 0, alt = 0;
    double pitch = 0, roll = 0, hdg = 0, spd = 0;
    bool   valid = false;
};

class UdpNode : public rclcpp::Node
{
public:
    UdpNode();
    ~UdpNode();

private:
    void recv_loop();   // UDP 수신 스레드
    void publish_cb();  // 발행 타이머

    int              sock_fd_ = -1;
    std::atomic<bool> running_{true};
    std::thread      recv_thread_;
    std::mutex       mutex_;
    FlightState      state_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
};
