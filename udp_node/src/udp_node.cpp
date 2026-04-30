#include "udp_node/udp_node.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

// ── 생성자 ────────────────────────────────────────────────────────────────────
UdpNode::UdpNode() : Node("udp_node")
{
    declare_parameter("listen_port",     50002);
    declare_parameter("publish_rate_hz", 50.0);

    int    port = get_parameter("listen_port").as_int();
    double rate = get_parameter("publish_rate_hz").as_double();

    // UDP 소켓 생성 + 바인딩
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        RCLCPP_FATAL(get_logger(), "소켓 생성 실패"); return;
    }
    int opt = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct timeval tv{1, 0};  // 수신 타임아웃 1초 (블로킹 방지)
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(static_cast<uint16_t>(port));

    if (bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_FATAL(get_logger(), "포트 %d 바인딩 실패", port);
        close(sock_fd_); sock_fd_ = -1; return;
    }

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/bootcamp/flight_state", rclcpp::QoS(10));

    auto period = std::chrono::duration<double>(1.0 / std::max(rate, 1.0));
    timer_ = create_wall_timer(period, std::bind(&UdpNode::publish_cb, this));

    recv_thread_ = std::thread(&UdpNode::recv_loop, this);

    RCLCPP_INFO(get_logger(),
        "udp_node | 포트 %d 수신 | %.0fHz → /bootcamp/flight_state", port, rate);
}

UdpNode::~UdpNode()
{
    running_ = false;
    if (sock_fd_ >= 0) close(sock_fd_);
    if (recv_thread_.joinable()) recv_thread_.join();
}

// ── UDP 수신 스레드 (sm_udp_receiver.cpp 참조) ────────────────────────────────
void UdpNode::recv_loop()
{
    char buf[4096];
    while (running_) {
        ssize_t n = recv(sock_fd_, buf, sizeof(buf) - 1, 0);
        if (n <= 0) continue;
        buf[n] = '\0';

        if (std::strncmp(buf, "HEADER=FLIGHT", 13) != 0) continue;

        FlightState s;
        double heading_mag = 0;
        // 참조: sm_udp_receiver.cpp sscanf 포맷 (HEADING_TRUE = 진북 방향)
        int ok = std::sscanf(buf,
            "HEADER=FLIGHT "
            "LATITUDE=%lf LONGITUDE=%lf ALTITUDE=%lf "
            "KOHLSMANN=%*f PITCH=%lf ROLL=%lf "
            "HEADING=%lf HEADING_TRUE=%lf "
            "VELOCITY_X=%*f VELOCITY_Y=%*f VELOCITY_Z=%*f "
            "TEMPERATURE=%*f AIRPRESSURE=%*f AIRDENSITY=%*f "
            "WIND_VELOCITY=%*f WIND_DIRECTION=%*f "
            "WIND_X=%*f WIND_Y=%*f WIND_Z=%*f "
            "AIRSPEED=%*f GROUNDSPEED=%lf",
            &s.lat, &s.lon, &s.alt,
            &s.pitch, &s.roll,
            &heading_mag, &s.hdg,   // hdg = HEADING_TRUE
            &s.spd);

        if (ok >= 7) {
            s.valid = true;
            std::lock_guard<std::mutex> lk(mutex_);
            state_ = s;
        }
    }
}

// ── 발행 타이머 ───────────────────────────────────────────────────────────────
void UdpNode::publish_cb()
{
    FlightState s;
    { std::lock_guard<std::mutex> lk(mutex_); s = state_; }
    if (!s.valid) return;

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {s.lat, s.lon, s.alt, s.pitch, s.roll, s.hdg, s.spd};
    pub_->publish(msg);
}
