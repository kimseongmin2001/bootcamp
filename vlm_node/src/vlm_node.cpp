#include "vlm_node/vlm_node.hpp"

#include <curl/curl.h>
#include <sstream>
#include <cstdio>

static size_t write_cb(void* p, size_t sz, size_t nm, std::string* buf)
{
    buf->append(static_cast<char*>(p), sz*nm); return sz*nm;
}

// ── 생성자 ────────────────────────────────────────────────────────────────────
VlmNode::VlmNode() : Node("vlm_node")
{
    declare_parameter("qwen_server_url",        std::string("http://localhost:8002"));
    declare_parameter("inference_interval_sec", 2.0);
    declare_parameter("max_new_tokens",         128);
    declare_parameter("jpeg_quality",           85);
    declare_parameter("request_timeout_sec",    20.0);

    qwen_url_    = get_parameter("qwen_server_url").as_string() + "/v1/chat/completions";
    double intv  = get_parameter("inference_interval_sec").as_double();
    max_tokens_  = get_parameter("max_new_tokens").as_int();
    jpeg_quality_= get_parameter("jpeg_quality").as_int();
    timeout_sec_ = get_parameter("request_timeout_sec").as_double();

    auto be  = rclcpp::QoS(1).best_effort();
    auto img = rclcpp::SensorDataQoS();

    sub_front_  = create_subscription<sensor_msgs::msg::Image>(
        "/bootcamp/front",  be,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m){ front_cb(m); });
    sub_bottom_ = create_subscription<sensor_msgs::msg::Image>(
        "/bootcamp/bottom", be,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m){ bottom_cb(m); });
    sub_state_  = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/bootcamp/flight_state", be,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m){ state_cb(m); });

    pub_panel_ = create_publisher<sensor_msgs::msg::Image>("/bootcamp/vlm_panel", img);

    // 추론 스레드 (별도 — ROS 콜백 블로킹 없음)
    infer_thread_ = std::thread([this, intv]() {
        while (rclcpp::ok() && running_) {
            std::this_thread::sleep_for(std::chrono::duration<double>(intv));
            infer_loop();
        }
    });

    panel_timer_ = create_wall_timer(std::chrono::milliseconds(500),
        std::bind(&VlmNode::publish_panel, this));

    RCLCPP_INFO(get_logger(),
        "vlm_node | %s | %.1fs 간격 → /bootcamp/vlm_panel",
        qwen_url_.c_str(), intv);
}

VlmNode::~VlmNode()
{
    running_ = false;
    if (infer_thread_.joinable()) infer_thread_.join();
}

// ── 콜백 ─────────────────────────────────────────────────────────────────────
void VlmNode::front_cb(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try {
        std::lock_guard<std::mutex> lk(mutex_);
        img_front_ = cv_bridge::toCvShare(msg,"bgr8")->image.clone();
    } catch (...) {}
}
void VlmNode::bottom_cb(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try {
        std::lock_guard<std::mutex> lk(mutex_);
        img_bottom_ = cv_bridge::toCvShare(msg,"bgr8")->image.clone();
    } catch (...) {}
}
void VlmNode::state_cb(std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    if (msg->data.size() < 7) return;
    std::lock_guard<std::mutex> lk(mutex_);
    flight_ = {msg->data[0],msg->data[1],msg->data[2],
               msg->data[3],msg->data[4],msg->data[5],
               msg->data[6], true};
}

// ── 추론 ─────────────────────────────────────────────────────────────────────
void VlmNode::infer_loop()
{
    cv::Mat front, bottom; FlightState f;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (img_front_.empty() || img_bottom_.empty() || !flight_.valid) return;
        front=img_front_.clone(); bottom=img_bottom_.clone(); f=flight_;
    }
    std::string resp = call_qwen(front, bottom, f);
    if (!resp.empty()) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_response_ = resp;
        RCLCPP_INFO(get_logger(), "Qwen: %s", resp.c_str());
    }
}

std::string VlmNode::call_qwen(const cv::Mat& front, const cv::Mat& bottom,
                                const FlightState& f)
{
    char prompt[512];
    std::snprintf(prompt, sizeof(prompt),
        "첫 번째 이미지는 드론 전면 카메라, 두 번째는 하방 카메라야. "
        "현재 위치 lat=%.4f lon=%.4f, 고도 %.0fm, 속도 %.1f m/s, 방향 %.0f도. "
        "드론이 지금 어떤 상황인지 한국어 한 줄로 간결하게 설명해줘.",
        f.lat, f.lon, f.alt, f.spd, f.hdg);

    std::ostringstream oss;
    oss << R"({"model":"Qwen2-VL-7B-TRT","messages":[{"role":"user","content":[)"
        << R"({"type":"image_url","image_url":{"url":"data:image/jpeg;base64,)"
        << encode_jpeg_b64(front,  jpeg_quality_) << R"("}},)"
        << R"({"type":"image_url","image_url":{"url":"data:image/jpeg;base64,)"
        << encode_jpeg_b64(bottom, jpeg_quality_) << R"("}},)"
        << R"({"type":"text","text":")" << prompt << R"("}]}],)"
        << R"("max_tokens":)" << max_tokens_ << R"(,"temperature":0.0,"stream":false})";

    std::string raw;
    try { raw = http_post(qwen_url_, oss.str(), timeout_sec_); }
    catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Qwen HTTP 오류: %s", e.what()); return "";
    }

    // "content":"..." 간단 파싱 (satellite_predictor.cpp 동일 패턴)
    auto pos = raw.find("\"content\":");
    if (pos == std::string::npos) return "";
    pos += 11;
    auto end = raw.find('"', pos);
    return (end != std::string::npos) ? raw.substr(pos, end-pos) : "";
}

// ── 패널 렌더링 ───────────────────────────────────────────────────────────────
void VlmNode::publish_panel()
{
    cv::Mat front, bottom; std::string text; FlightState f;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        front  = img_front_.empty()  ? cv::Mat() : img_front_.clone();
        bottom = img_bottom_.empty() ? cv::Mat() : img_bottom_.clone();
        text   = last_response_; f = flight_;
    }
    cv::Mat panel = render_panel(front, bottom, text, f);
    auto msg = cv_bridge::CvImage({}, "bgr8", panel).toImageMsg();
    msg->header.stamp = now();
    pub_panel_->publish(*msg);
}

cv::Mat VlmNode::render_panel(const cv::Mat& front, const cv::Mat& bottom,
                               const std::string& text, const FlightState& f)
{
    constexpr int PW=1280, PH=480, TW=560, TH=315, TEXT_H=130;
    cv::Mat panel(PH, PW, CV_8UC3, cv::Scalar(18,18,18));

    // 썸네일 (전면 / 배면)
    const cv::Mat* imgs[2]   = {&front,        &bottom};
    const char*    labels[2] = {"전면 카메라", "배면 카메라"};
    for (int i = 0; i < 2; ++i) {
        int x0=i*(TW+20)+20, y0=20;
        cv::Mat thumb;
        if (!imgs[i]->empty()) cv::resize(*imgs[i], thumb, {TW,TH});
        else {
            thumb = cv::Mat(TH,TW,CV_8UC3,cv::Scalar(55,55,55));
            cv::putText(thumb,"카메라 없음",{TW/2-65,TH/2},
                cv::FONT_HERSHEY_SIMPLEX,0.7,{120,120,120},2,cv::LINE_AA);
        }
        thumb.copyTo(panel(cv::Rect(x0,y0,TW,TH)));
        cv::rectangle(panel,{x0,y0+TH-28},{x0+TW,y0+TH},{0,0,0},-1);
        cv::putText(panel,labels[i],{x0+8,y0+TH-8},
            cv::FONT_HERSHEY_SIMPLEX,0.6,{200,200,200},1,cv::LINE_AA);
    }

    // Qwen 응답 패널
    int ty0 = PH - TEXT_H;
    cv::Mat ov = panel.clone();
    cv::rectangle(ov,{0,ty0},{PW,PH},{8,8,8},-1);
    cv::addWeighted(ov,0.88,panel,0.12,0,panel);
    cv::line(panel,{0,ty0},{PW,ty0},{60,60,60},1);
    cv::putText(panel,"[ Qwen2-VL-7B 분석 ]",{14,ty0+24},
        cv::FONT_HERSHEY_SIMPLEX,0.58,{80,180,255},1,cv::LINE_AA);

    // 텍스트 줄바꿈
    std::vector<std::string> lines;
    std::string line, word;
    std::istringstream iss(text);
    while (iss >> word) {
        std::string c = line.empty() ? word : line+" "+word;
        if (c.size() > 58) { if (!line.empty()) lines.push_back(line); line=word; }
        else line=c;
    }
    if (!line.empty()) lines.push_back(line);
    for (int i=0; i<std::min((int)lines.size(),3); ++i)
        cv::putText(panel,lines[i],{14,ty0+52+i*28},
            cv::FONT_HERSHEY_SIMPLEX,0.68,{240,240,240},1,cv::LINE_AA);

    // 비행 상태 바
    if (f.valid) {
        char info[256];
        std::snprintf(info,sizeof(info),
            "lat=%.4f  lon=%.4f  alt=%.0fm  spd=%.1fm/s  hdg=%.0f°",
            f.lat,f.lon,f.alt,f.spd,f.hdg);
        cv::rectangle(panel,{0,PH-22},{PW,PH},{0,0,0},-1);
        cv::putText(panel,info,{14,PH-6},
            cv::FONT_HERSHEY_SIMPLEX,0.47,{140,210,140},1,cv::LINE_AA);
    }
    return panel;
}

// ── JPEG base64 인코딩 (satellite_predictor.cpp 동일 패턴) ───────────────────
std::string VlmNode::encode_jpeg_b64(const cv::Mat& img, int quality)
{
    std::vector<uchar> buf;
    cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, quality});
    static const char B64[]=
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out; out.reserve((buf.size()/3+1)*4);
    for (size_t i=0; i<buf.size(); i+=3) {
        unsigned b0=buf[i], b1=(i+1<buf.size())?buf[i+1]:0u,
                             b2=(i+2<buf.size())?buf[i+2]:0u;
        out+=B64[b0>>2]; out+=B64[((b0&3)<<4)|(b1>>4)];
        out+=(i+1<buf.size())?B64[((b1&0xf)<<2)|(b2>>6)]:'=';
        out+=(i+2<buf.size())?B64[b2&0x3f]:'=';
    }
    return out;
}

// ── HTTP POST ─────────────────────────────────────────────────────────────────
std::string VlmNode::http_post(const std::string& url,
                                const std::string& body, double timeout)
{
    CURL* curl = curl_easy_init();
    if (!curl) throw std::runtime_error("curl_easy_init 실패");
    std::string resp;
    curl_slist* hdrs = nullptr;
    hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_URL,           url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,    body.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,    hdrs);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA,     &resp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,       static_cast<long>(timeout));
    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(hdrs); curl_easy_cleanup(curl);
    if (res != CURLE_OK)
        throw std::runtime_error(std::string("curl: ")+curl_easy_strerror(res));
    return resp;
}
