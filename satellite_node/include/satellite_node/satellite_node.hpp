#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "satellite_node/tile_fetcher.hpp"

struct FlightState {
    double lat=0, lon=0, alt=0, pitch=0, roll=0, hdg=0, spd=0;
    bool   valid = false;
};

class SatelliteNode : public rclcpp::Node
{
public:
    SatelliteNode();
    ~SatelliteNode();

private:
    // callbacks
    void state_cb(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    void render_cb();
    void tile_prefetch_loop();

    // RViz publishing
    void publish_markers(const FlightState& f,
                         double east, double north);
    void publish_ground_cloud(const TileCanvas& tc,
                              double ref_lat, double ref_lon);

    // rqt image (vlm_node용, 유지)
    void publish_image(const TileCanvas& tc, const FlightState& f,
                       double east, double north);

    // GPS → local ENU (flat-earth)
    void gps_to_enu(double lat, double lon,
                    double& east, double& north) const;

    std::unique_ptr<TileFetcher> fetcher_;

    std::string mesh_uri_;     // file:///...joby_s4.stl

    // GPS reference (set at first valid fix, then fixed)
    bool   ref_set_ = false;
    double ref_lat_ = 0, ref_lon_ = 0;

    // flight state
    std::mutex  flight_mutex_;
    FlightState flight_;

    // canvas cache
    std::mutex canvas_mutex_;
    TileCanvas  latest_canvas_;
    bool        canvas_ready_ = false;

    // trajectory (ENU positions)
    std::mutex                    traj_mutex_;
    std::deque<geometry_msgs::msg::Point> trajectory_;
    static constexpr std::size_t TRAJ_MAX = 300;

    int  zoom_, grid_;

    std::atomic<bool> prefetch_running_{true};
    std::thread       prefetch_thread_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
