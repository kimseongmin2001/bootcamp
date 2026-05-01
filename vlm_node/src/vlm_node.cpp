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

    // 한글 렌더링용 FreeType2 초기화
    ft2_ = cv::freetype::createFreeType2();
    const std::string font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf";
    try {
        ft2_->loadFontData(font_path, 0);
        RCLCPP_INFO(get_logger(), "FreeType2 font loaded: %s", font_path.c_str());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "한글 폰트 로드 실패: %s", font_path.c_str());
        ft2_ = nullptr;
    }

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
    // 방향을 한국어 방위로 변환
    const char* dir_str = "북";
    double hdg = f.hdg;
    if      (hdg <  22.5 || hdg >= 337.5) dir_str = "북";
    else if (hdg <  67.5)                 dir_str = "북동";
    else if (hdg < 112.5)                 dir_str = "동";
    else if (hdg < 157.5)                 dir_str = "남동";
    else if (hdg < 202.5)                 dir_str = "남";
    else if (hdg < 247.5)                 dir_str = "남서";
    else if (hdg < 292.5)                 dir_str = "서";
    else                                  dir_str = "북서";

    char prompt[1024];
    std::snprintf(prompt, sizeof(prompt),
        "첫 번째 이미지는 드론 전면 카메라, 두 번째는 하방 카메라야. "
        "현재 GPS: lat=%.5f lon=%.5f, 고도 %.0fm, 속도 %.1fm/s, 방향 %s(%.0f도). "
        "아래 형식으로 정확히 3줄만 한국어로 답해. 각 줄 앞에 태그를 붙여: "
        "[위치] 현재 어느 지역/랜드마크 위를 비행 중인지 (예: 여의도 국회의사당 북쪽) "
        "[전방] 전면 카메라에 보이는 것과 진행 방향의 지형/건물 (예: 한강과 원효대교가 보임) "
        "[전망] 현재 방향으로 곧 나타날 주요 랜드마크나 건물 (예: 63빌딩이 약 1km 전방)",
        f.lat, f.lon, f.alt, f.spd, dir_str, hdg);

    std::ostringstream oss;
    oss << R"({"model":"Qwen2-VL-7B-TRT","messages":[{"role":"user","content":[)"
        << R"({"type":"image_url","image_url":{"url":"data:image/jpeg;base64,)"
        << encode_jpeg_b64(front,  jpeg_quality_) << R"("}},)"
        << R"({"type":"image_url","image_url":{"url":"data:image/jpeg;base64,)"
        << encode_jpeg_b64(bottom, jpeg_quality_) << R"("}},)"
        << R"({"type":"text","text":")" << prompt << R"("}]}],)"
        << R"("max_tokens":)" << max_tokens_ << R"(,"temperature":0.1,"stream":false})";

    std::string raw;
    try { raw = http_post(qwen_url_, oss.str(), timeout_sec_); }
    catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Qwen HTTP 오류: %s", e.what()); return "";
    }

    // choices[0].message.content 파싱
    // "message" 다음에 나오는 "content": 를 찾아야 정확함
    auto msg_pos = raw.find("\"message\"");
    if (msg_pos == std::string::npos) msg_pos = 0;
    auto pos = raw.find("\"content\":", msg_pos);
    if (pos == std::string::npos) return "";
    pos += 10; // skip "content":

    // 여는 " 찾기 (공백 있을수도 없을수도)
    while (pos < raw.size() && raw[pos] != '"') ++pos;
    if (pos >= raw.size()) return "";
    ++pos; // skip opening "

    // 닫는 " 까지 수집 (이스케이프 처리)
    std::string content;
    for (size_t i = pos; i < raw.size(); ++i) {
        if (raw[i] == '\\' && i+1 < raw.size()) {
            char c = raw[i+1];
            if      (c == 'n')  { content += '\n'; ++i; }
            else if (c == '"')  { content += '"';  ++i; }
            else if (c == '\\') { content += '\\'; ++i; }
            else if (c == 't')  { content += '\t'; ++i; }
            else                { content += c;    ++i; }
        } else if (raw[i] == '"') { break; }
        else { content += raw[i]; }
    }
    return content;
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

// [태그] 내용 파싱 헬퍼 — 다음 [태그] 또는 줄바꿈까지 추출
static std::string extract_tag(const std::string& text, const std::string& tag)
{
    auto pos = text.find(tag);
    if (pos == std::string::npos) return "";
    pos += tag.size();

    // 다음 [ 또는 \n 중 가까운 것에서 종료
    size_t end = text.size();
    for (size_t i = pos; i < text.size(); ++i) {
        if (text[i] == '\n') { end = i; break; }
        // 다음 태그 시작 "[위치]", "[전방]", "[전망]" 패턴
        if (text[i] == '[' && i != pos) { end = i; break; }
    }
    std::string val = text.substr(pos, end - pos);
    // 앞뒤 공백·콜론 제거
    size_t s = val.find_first_not_of(" \t:");
    size_t e = val.find_last_not_of(" \t\r:");
    return (s == std::string::npos) ? "" : val.substr(s, e - s + 1);
}

cv::Mat VlmNode::render_panel(const cv::Mat& front, const cv::Mat& bottom,
                               const std::string& text, const FlightState& f)
{
    constexpr int PW=1280, PH=560, TW=560, TH=315;
    cv::Mat panel(PH, PW, CV_8UC3, cv::Scalar(15,15,20));

    // FreeType 사용 가능 여부
    bool use_ft = (ft2_ != nullptr);
    auto put_text = [&](cv::Mat& img, const std::string& s,
                        cv::Point pt, int px, cv::Scalar col, int thick=1) {
        if (use_ft)
            ft2_->putText(img, s, pt, px, col, thick, cv::LINE_AA, false);
        else
            cv::putText(img, s, pt, cv::FONT_HERSHEY_SIMPLEX,
                        px / 28.0, col, thick, cv::LINE_AA);
    };

    // ── 썸네일 (전면 / 배면) ─────────────────────────────────────────────
    const cv::Mat* imgs[2]   = {&front,        &bottom};
    const char*    labels[2] = {"전면 카메라", "배면 카메라"};
    for (int i = 0; i < 2; ++i) {
        int x0=i*(TW+20)+20, y0=8;
        cv::Mat thumb;
        if (!imgs[i]->empty()) {
            // 사방 5% 크롭 (WINDOW 타이틀바 등 제거)
            const cv::Mat& src = *imgs[i];
            int cw = src.cols, ch = src.rows;
            int cx = static_cast<int>(cw * 0.05);
            int cy = static_cast<int>(ch * 0.05);
            cv::Mat cropped = src(cv::Rect(cx, cy, cw-2*cx, ch-2*cy));
            cv::resize(cropped, thumb, {TW, TH});
        } else {
            thumb = cv::Mat(TH,TW,CV_8UC3,cv::Scalar(40,40,50));
            put_text(thumb, "카메라 없음", {TW/2-65, TH/2}, 22, {100,100,120});
        }
        // 라벨을 thumb 내부 좌상단에 직접 오버레이 (잘릴 일 없음)
        cv::rectangle(thumb, {0,0}, {TW, 34}, {5,5,8}, -1);
        put_text(thumb, labels[i], {10, 26}, 22, {200,210,255});
        thumb.copyTo(panel(cv::Rect(x0, y0, TW, TH)));
        cv::rectangle(panel,{x0,y0},{x0+TW,y0+TH},{60,60,80},1);
    }

    // ── 정보 패널 ────────────────────────────────────────────────────────
    int iy0 = TH + 18;
    cv::rectangle(panel,{0,iy0},{PW,PH},{10,12,18},-1);
    cv::line(panel,{0,iy0},{PW,iy0},{50,80,120},2);

    struct Section { const char* tag_label; const char* tag_key;
                     cv::Scalar label_color; cv::Scalar text_color; };
    Section sections[] = {
        {"▶ 현재 위치", "[위치]", {100,220,255}, {220,240,255}},
        {"▶ 전  방",   "[전방]", {100,255,160}, {210,255,230}},
        {"▶ 전  망",   "[전망]", {255,200, 80}, {255,235,180}},
    };

    bool has_tags = text.find("[위치]") != std::string::npos ||
                    text.find("[전방]") != std::string::npos ||
                    text.find("[전망]") != std::string::npos;

    if (has_tags) {
        int row_h = 56;
        for (int i = 0; i < 3; ++i) {
            int rx = 14, ry = iy0 + 14 + i * row_h;
            cv::rectangle(panel,{rx,ry},{rx+110,ry+28},{20,25,35},-1);
            put_text(panel, sections[i].tag_label, {rx+4, ry+22}, 18, sections[i].label_color);
            std::string val = extract_tag(text, sections[i].tag_key);
            if (val.empty()) val = "정보 없음";
            put_text(panel, val, {rx+120, ry+22}, 20, sections[i].text_color);
            if (i < 2)
                cv::line(panel,{rx,ry+row_h-4},{PW-14,ry+row_h-4},{35,40,55},1);
        }
    } else if (!text.empty()) {
        put_text(panel, "[ Qwen2-VL-7B ]", {14, iy0+28}, 18, {80,160,255});
        // 줄 단위로 출력
        std::istringstream iss(text);
        std::string ln; int row = 0;
        while (std::getline(iss, ln) && row < 4) {
            if (!ln.empty()) put_text(panel, ln, {14, iy0+56+row*30}, 19, {230,230,230});
            ++row;
        }
    } else {
        put_text(panel, "Qwen 응답 대기 중...", {14, iy0+50}, 20, {80,80,100});
    }

    // ── 비행 상태 바 ─────────────────────────────────────────────────────
    cv::rectangle(panel,{0,PH-28},{PW,PH},{6,8,14},-1);
    cv::line(panel,{0,PH-28},{PW,PH-28},{40,50,70},1);
    if (f.valid) {
        char info[256];
        std::snprintf(info,sizeof(info),
            "LAT %.5f   LON %.5f   ALT %.0f m   SPD %.1f m/s   HDG %.0f deg",
            f.lat,f.lon,f.alt,f.spd,f.hdg);
        cv::putText(panel,info,{14,PH-8},
            cv::FONT_HERSHEY_SIMPLEX,0.46,{120,200,120},1,cv::LINE_AA);
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
