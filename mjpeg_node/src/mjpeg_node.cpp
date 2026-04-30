#include "mjpeg_node/mjpeg_node.hpp"
#include <curl/curl.h>

// ── MjpegStream ───────────────────────────────────────────────────────────────
struct CurlCtx { MjpegStream* stream; std::vector<uint8_t> buf; };

MjpegStream::MjpegStream(std::string url, std::string topic,
                          rclcpp::Node* node, int w, int h)
    : url_(std::move(url)), w_(w), h_(h), node_(node)
{
    pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS());
    thread_ = std::thread(&MjpegStream::stream_loop, this);
}

MjpegStream::~MjpegStream()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

// libcurl 콜백 — 수신 바이트를 누적하고 JPEG 프레임 추출 후 즉시 발행
size_t MjpegStream::curl_write_cb(char* ptr, size_t size, size_t nmemb, void* ud)
{
    auto* ctx = static_cast<CurlCtx*>(ud);
    ctx->buf.insert(ctx->buf.end(),
        reinterpret_cast<uint8_t*>(ptr),
        reinterpret_cast<uint8_t*>(ptr) + size * nmemb);

    cv::Mat frame;
    while (ctx->stream->extract_frame(ctx->buf, frame)) {
        cv::resize(frame, frame, {ctx->stream->w_, ctx->stream->h_});
        auto msg = cv_bridge::CvImage({}, "bgr8", frame).toImageMsg();
        msg->header.stamp = ctx->stream->node_->get_clock()->now();
        ctx->stream->pub_->publish(*msg);
    }
    return size * nmemb;
}

// SOI(FFD8) ~ EOI(FFD9) 탐색으로 완전한 JPEG 프레임 추출
bool MjpegStream::extract_frame(std::vector<uint8_t>& buf, cv::Mat& out)
{
    const uint8_t SOI[2] = {0xFF, 0xD8};
    const uint8_t EOI[2] = {0xFF, 0xD9};

    auto soi = std::search(buf.begin(), buf.end(), SOI, SOI+2);
    if (soi == buf.end()) { buf.clear(); return false; }

    auto eoi = std::search(soi+2, buf.end(), EOI, EOI+2);
    if (eoi == buf.end()) return false;
    eoi += 2;

    cv::Mat img = cv::imdecode(
        std::vector<uint8_t>(soi, eoi), cv::IMREAD_COLOR);
    buf.erase(buf.begin(), eoi);
    if (img.empty()) return false;

    out = std::move(img);
    return true;
}

void MjpegStream::stream_loop()
{
    while (running_) {
        CURL* curl = curl_easy_init();
        if (!curl) { std::this_thread::sleep_for(std::chrono::seconds(2)); continue; }

        CurlCtx ctx{this, {}};
        curl_easy_setopt(curl, CURLOPT_URL,             url_.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,   curl_write_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,       &ctx);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT,  5L);
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME,  10L);
        curl_easy_setopt(curl, CURLOPT_TCP_NODELAY,     1L);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if (running_ && res != CURLE_OK)
            RCLCPP_WARN(node_->get_logger(),
                "MJPEG 끊김 [%s] — 2초 후 재연결", url_.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ── MjpegNode ─────────────────────────────────────────────────────────────────
MjpegNode::MjpegNode() : Node("mjpeg_node")
{
    declare_parameter("front_url",  std::string("http://192.168.0.13:8083"));
    declare_parameter("bottom_url", std::string("http://192.168.0.13:8084"));

    front_  = std::make_unique<MjpegStream>(
        get_parameter("front_url").as_string(),  "/bootcamp/front",  this, 1280, 720);
    bottom_ = std::make_unique<MjpegStream>(
        get_parameter("bottom_url").as_string(), "/bootcamp/bottom", this, 1280, 720);

    RCLCPP_INFO(get_logger(), "mjpeg_node | front=%s | bottom=%s",
        get_parameter("front_url").as_string().c_str(),
        get_parameter("bottom_url").as_string().c_str());
}

MjpegNode::~MjpegNode() = default;
