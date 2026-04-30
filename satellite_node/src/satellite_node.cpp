#include "satellite_node/satellite_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>
#include <filesystem>

static constexpr double DEG2RAD    = M_PI / 180.0;
static constexpr double EARTH_R    = 6371000.0;
static constexpr double DEFAULT_LAT = 37.283195;
static constexpr double DEFAULT_LON = 127.025901;

// rqt 이미지 출력 크기
static constexpr int IMG_W = 1280;
static constexpr int IMG_H =  960;

// ── 유틸 ──────────────────────────────────────────────────────────────────────
static geometry_msgs::msg::Point make_pt(double x, double y, double z)
{
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}
static std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a=1.f)
{
    std_msgs::msg::ColorRGBA c;
    c.r=r; c.g=g; c.b=b; c.a=a;
    return c;
}

// ── 생성자 ────────────────────────────────────────────────────────────────────
SatelliteNode::SatelliteNode() : Node("satellite_node")
{
    declare_parameter("tile_zoom",       18);
    declare_parameter("grid_tiles",       7);
    declare_parameter("publish_rate_hz",  1.0);
    declare_parameter("tile_cache_dir",
        std::string("/home/kimseongmin/.cache/bootcamp_tiles"));
    declare_parameter("mesh_path", std::string(""));  // joby_s4.stl 절대경로

    zoom_  = get_parameter("tile_zoom").as_int();
    grid_  = get_parameter("grid_tiles").as_int();
    double rate  = get_parameter("publish_rate_hz").as_double();
    std::string cache = get_parameter("tile_cache_dir").as_string();

    // mesh URI (RViz MESH_RESOURCE에 필요)
    std::string mesh_path = get_parameter("mesh_path").as_string();
    if (mesh_path.empty()) {
        // 패키지 내부 기본 경로 자동 탐색
        namespace fs = std::filesystem;
        fs::path pkg = fs::path(__FILE__).parent_path().parent_path();
        mesh_path = (pkg / "meshes" / "joby_s4.stl").string();
    }
    mesh_uri_ = "file://" + mesh_path;
    RCLCPP_INFO(get_logger(), "Joby mesh: %s", mesh_uri_.c_str());

    fetcher_ = std::make_unique<TileFetcher>(cache, 8.0);

    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/bootcamp/flight_state", rclcpp::QoS(10),
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
            state_cb(msg); });

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/bootcamp/markers", rclcpp::QoS(10));
    cloud_pub_   = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/bootcamp/ground_map", rclcpp::SensorDataQoS());
    image_pub_   = create_publisher<sensor_msgs::msg::Image>(
        "/bootcamp/satellite_map", rclcpp::SensorDataQoS());

    prefetch_thread_ = std::thread(&SatelliteNode::tile_prefetch_loop, this);

    auto period = std::chrono::duration<double>(1.0 / std::max(rate, 0.1));
    timer_ = create_wall_timer(period, std::bind(&SatelliteNode::render_cb, this));

    RCLCPP_INFO(get_logger(),
        "satellite_node | zoom=%d | grid=%d | %.1fHz", zoom_, grid_, rate);
}

SatelliteNode::~SatelliteNode()
{
    prefetch_running_ = false;
    if (prefetch_thread_.joinable()) prefetch_thread_.join();
}

void SatelliteNode::state_cb(
    std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    if (msg->data.size() < 7) return;
    std::lock_guard<std::mutex> lk(flight_mutex_);
    flight_ = {msg->data[0], msg->data[1], msg->data[2],
               msg->data[3], msg->data[4], msg->data[5],
               msg->data[6], true};
}

void SatelliteNode::tile_prefetch_loop()
{
    while (rclcpp::ok() && prefetch_running_) {
        FlightState f;
        { std::lock_guard<std::mutex> lk(flight_mutex_); f = flight_; }
        double lat = f.valid ? f.lat : DEFAULT_LAT;
        double lon = f.valid ? f.lon : DEFAULT_LON;
        try {
            TileCanvas tc = fetcher_->build_canvas(lat, lon, zoom_, grid_);
            std::lock_guard<std::mutex> lk(canvas_mutex_);
            latest_canvas_ = std::move(tc);
            canvas_ready_  = true;
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "타일 오류: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// ── GPS → ENU (flat-earth) ────────────────────────────────────────────────────
void SatelliteNode::gps_to_enu(double lat, double lon,
                                double& east, double& north) const
{
    double dlat = (lat - ref_lat_) * DEG2RAD;
    double dlon = (lon - ref_lon_) * DEG2RAD;
    north = dlat * EARTH_R;
    east  = dlon * EARTH_R * std::cos(ref_lat_ * DEG2RAD);
}

// ── RViz 마커 발행 ────────────────────────────────────────────────────────────
void SatelliteNode::publish_markers(const FlightState& f,
                                    double east, double north)
{
    visualization_msgs::msg::MarkerArray ma;
    auto now_stamp = now();
    const std::string frame = "map";

    auto base_marker = [&](int id, int type) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp    = now_stamp;
        m.ns   = "bootcamp";
        m.id   = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        return m;
    };

    // ── 0: Joby S4 메시 ───────────────────────────────────────────────────
    {
        auto m = base_marker(0, visualization_msgs::msg::Marker::MESH_RESOURCE);
        m.mesh_resource = mesh_uri_;
        m.mesh_use_embedded_materials = false;

        // 위치: ENU (east, north, altitude)
        m.pose.position = make_pt(east, north, f.alt);

        // 방향: heading (yaw around Z in ENU)
        // ENU에서 +X = East, +Y = North, +Z = Up
        // 기체 +X = nose → ENU yaw: 0° = East, 90° = North
        // MSFS heading: 0° = North, 90° = East → yaw = 90° - hdg
        double yaw = (90.0 - f.hdg) * DEG2RAD;
        m.pose.orientation.z = std::sin(yaw / 2.0);
        m.pose.orientation.w = std::cos(yaw / 2.0);

        // pitch / roll 적용
        // 간단히 yaw만 적용 (pitch/roll은 선택)

        m.scale.x = m.scale.y = m.scale.z = 1.0;
        m.color = make_color(0.85f, 0.90f, 0.95f, 1.0f);   // 밝은 회백색
        ma.markers.push_back(m);
    }

    // ── 1: 수직 하강선 (기체 → 지면) ────────────────────────────────────
    {
        auto m = base_marker(1, visualization_msgs::msg::Marker::LINE_STRIP);
        m.scale.x = 1.5;
        m.color   = make_color(0.0f, 0.85f, 1.0f, 0.8f);   // 시안
        m.points.push_back(make_pt(east, north, f.alt));
        m.points.push_back(make_pt(east, north, 0.0));
        ma.markers.push_back(m);
    }

    // ── 2: 지면 그림자 원 ─────────────────────────────────────────────────
    {
        auto m = base_marker(2, visualization_msgs::msg::Marker::CYLINDER);
        m.pose.position = make_pt(east, north, 0.05);
        m.scale.x = 12.0; m.scale.y = 12.0; m.scale.z = 0.1;
        m.color   = make_color(0.0f, 1.0f, 0.4f, 0.35f);   // 반투명 그린
        ma.markers.push_back(m);
    }

    // ── 3: 고도 텍스트 ────────────────────────────────────────────────────
    {
        auto m = base_marker(3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.0f m AGL", f.alt);
        m.text = buf;
        m.pose.position = make_pt(east + 15, north + 15, f.alt / 2.0);
        m.scale.z = 8.0;
        m.color   = make_color(0.0f, 0.85f, 1.0f, 1.0f);
        ma.markers.push_back(m);
    }

    // ── 4: 비행 궤적 ──────────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(traj_mutex_);
        // 현재 위치 추가
        trajectory_.push_back(make_pt(east, north, f.alt));
        if (trajectory_.size() > TRAJ_MAX) trajectory_.pop_front();

        if (trajectory_.size() > 1) {
            auto m = base_marker(4, visualization_msgs::msg::Marker::LINE_STRIP);
            m.scale.x = 2.0;
            m.color   = make_color(1.0f, 0.55f, 0.0f, 0.9f);   // 주황
            m.points.assign(trajectory_.begin(), trajectory_.end());
            ma.markers.push_back(m);
        }
    }

    // ── 5: 속도 벡터 화살표 ──────────────────────────────────────────────
    if (f.spd > 0.5) {
        auto m = base_marker(5, visualization_msgs::msg::Marker::ARROW);
        double yaw  = (90.0 - f.hdg) * DEG2RAD;
        double vx = f.spd * std::cos(yaw);
        double vy = f.spd * std::sin(yaw);
        m.points.push_back(make_pt(east, north, f.alt));
        m.points.push_back(make_pt(east + vx*3, north + vy*3, f.alt));
        m.scale.x = 2.5; m.scale.y = 5.0; m.scale.z = 5.0;
        m.color   = make_color(1.0f, 1.0f, 0.0f, 0.9f);        // 노랑
        ma.markers.push_back(m);
    }

    // ── 6: HUD 텍스트 패널 ────────────────────────────────────────────────
    {
        auto m = base_marker(6, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "LAT  %.5f\nLON  %.5f\nALT  %.1f m\nSPD  %.1f m/s\nHDG  %.1f deg",
            f.valid ? f.lat : DEFAULT_LAT,
            f.valid ? f.lon : DEFAULT_LON,
            f.alt, f.spd, f.hdg);
        m.text = buf;
        m.pose.position = make_pt(east + 50, north, f.alt + 20);
        m.scale.z = 6.0;
        m.color = f.valid ? make_color(0.6f, 1.0f, 0.6f)
                          : make_color(1.0f, 0.8f, 0.2f);
        ma.markers.push_back(m);
    }

    markers_pub_->publish(ma);
}

// ── 위성 지도 → PointCloud2 (지면 텍스처) ─────────────────────────────────────
void SatelliteNode::publish_ground_cloud(const TileCanvas& tc,
                                          double ref_lat, double ref_lon)
{
    // 다운샘플: 256×256 으로 축소
    cv::Mat small;
    cv::resize(tc.image, small, {256, 256}, 0, 0, cv::INTER_AREA);

    const int rows = small.rows, cols = small.cols;
    const int N = rows * cols;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp    = now();
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width  = N;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(N);

    // canvas 전체 크기 (픽셀)
    int cs = tc.image.cols;   // 타일 캔버스 크기

    // 캔버스 네 모서리의 GPS 좌표를 ENU로 변환하여 스케일 계산
    // top-left GPS: (tc.top_lat, tc.left_lon)
    // 픽셀 당 미터
    double mpp = tc.meters_per_pixel;

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_rgb(cloud, "rgb");

    // 캔버스 (0,0) → ENU
    double tl_east, tl_north;
    double dlat = (tc.top_lat - ref_lat) * DEG2RAD;
    double dlon = (tc.left_lon - ref_lon) * DEG2RAD;
    tl_north = dlat * EARTH_R;
    tl_east  = dlon * EARTH_R * std::cos(ref_lat * DEG2RAD);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            // 캔버스 픽셀 좌표 → ENU
            // x(canvas) = col/cols * cs → east
            // y(canvas) = row/rows * cs → south (note: canvas Y is south)
            double fx = static_cast<double>(c) / cols * cs;
            double fy = static_cast<double>(r) / rows * cs;

            float east_m  = static_cast<float>(tl_east  + fx * mpp);
            float north_m = static_cast<float>(tl_north - fy * mpp);  // Y 반전

            *it_x = east_m;
            *it_y = north_m;
            *it_z = 0.0f;

            // BGR → packed RGB float
            cv::Vec3b px = small.at<cv::Vec3b>(r, c);
            uint32_t rgb = ((uint32_t)px[2] << 16) |
                           ((uint32_t)px[1] <<  8) |
                           ((uint32_t)px[0]);
            float rgb_f;
            std::memcpy(&rgb_f, &rgb, sizeof(float));
            *it_rgb = rgb_f;

            ++it_x; ++it_y; ++it_z; ++it_rgb;
        }
    }

    cloud_pub_->publish(cloud);
}

// ── rqt 용 2D 이미지 (vlm_node 입력) ─────────────────────────────────────────
void SatelliteNode::publish_image(const TileCanvas& tc,
                                   const FlightState& f,
                                   double east, double north)
{
    (void)east; (void)north;

    int n  = 1 << zoom_;
    double lat = f.valid ? f.lat : DEFAULT_LAT;
    double lon = f.valid ? f.lon : DEFAULT_LON;
    int px = static_cast<int>((lon - tc.left_lon) / 360.0 * n * 256);
    double lr = lat * DEG2RAD, tr = tc.top_lat * DEG2RAD;
    double pyf = (std::log(std::tan(tr)+1.0/std::cos(tr)) -
                  std::log(std::tan(lr)+1.0/std::cos(lr))) / M_PI;
    int py = static_cast<int>(pyf / 2.0 * n * 256);

    cv::Mat canvas = tc.image.clone();
    // 드론 마커
    if (px >= 0 && py >= 0 && px < canvas.cols && py < canvas.rows) {
        double r = f.hdg * DEG2RAD;
        int ex = px + static_cast<int>(55 * std::sin(r));
        int ey = py - static_cast<int>(55 * std::cos(r));
        cv::circle(canvas, {px,py}, 14, {0,220,0}, -1);
        cv::circle(canvas, {px,py}, 14, {255,255,255}, 2);
        cv::arrowedLine(canvas, {px,py}, {ex,ey},
            {0,255,255}, 3, cv::LINE_AA, 0, 0.35);
    }

    cv::Mat out;
    cv::resize(canvas, out, {IMG_W, IMG_H});
    auto msg = cv_bridge::CvImage({}, "bgr8", out).toImageMsg();
    msg->header.stamp    = now();
    msg->header.frame_id = "map";
    image_pub_->publish(*msg);
}

// ── 메인 렌더 콜백 ────────────────────────────────────────────────────────────
void SatelliteNode::render_cb()
{
    TileCanvas tc;
    {
        std::lock_guard<std::mutex> lk(canvas_mutex_);
        if (!canvas_ready_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                "위성 타일 다운로드 중...");
            return;
        }
        tc = latest_canvas_;
    }

    FlightState f;
    { std::lock_guard<std::mutex> lk(flight_mutex_); f = flight_; }

    double lat = f.valid ? f.lat : DEFAULT_LAT;
    double lon = f.valid ? f.lon : DEFAULT_LON;

    // GPS 기준점 (처음 한 번만 설정)
    if (!ref_set_) {
        ref_lat_ = lat; ref_lon_ = lon;
        ref_set_ = true;
        RCLCPP_INFO(get_logger(),
            "ENU 기준점 설정: lat=%.5f lon=%.5f", ref_lat_, ref_lon_);
    }

    double east, north;
    gps_to_enu(lat, lon, east, north);

    // RViz 마커 발행
    publish_markers(f, east, north);

    // 위성 지도 PointCloud2 (1Hz마다 갱신하면 너무 무거우므로 5초마다)
    static int cloud_skip = 0;
    if (cloud_skip++ % 5 == 0)
        publish_ground_cloud(tc, ref_lat_, ref_lon_);

    // rqt / vlm_node용 이미지
    publish_image(tc, f, east, north);
}
