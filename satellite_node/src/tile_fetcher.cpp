#include "satellite_node/tile_fetcher.hpp"

#include <curl/curl.h>
#include <filesystem>
#include <cmath>
#include <future>
#include <vector>

namespace fs = std::filesystem;
static constexpr int    TILE_SZ = 256;
static constexpr double DEG2RAD = M_PI / 180.0;

std::pair<int,int> lat_lon_to_tile(double lat, double lon, int zoom)
{
    int n = 1 << zoom;
    int x = static_cast<int>((lon + 180.0) / 360.0 * n);
    double lr = lat * DEG2RAD;
    int y = static_cast<int>(
        (1.0 - std::log(std::tan(lr) + 1.0/std::cos(lr)) / M_PI) / 2.0 * n);
    return {x, y};
}

std::pair<double,double> tile_to_lat_lon(int x, int y, int zoom)
{
    int n = 1 << zoom;
    double lon = static_cast<double>(x) / n * 360.0 - 180.0;
    double lat = std::atan(std::sinh(M_PI*(1.0-2.0*y/n))) / DEG2RAD;
    return {lat, lon};
}

double meters_per_pixel(double lat, int zoom)
{
    return 156543.03392 / (1 << zoom) * std::cos(lat * DEG2RAD);
}

static size_t write_cb(void* ptr, size_t size, size_t nmemb, std::string* buf)
{
    buf->append(static_cast<char*>(ptr), size * nmemb);
    return size * nmemb;
}

TileFetcher::TileFetcher(std::string cache_dir, double timeout_sec,
                         std::string local_tiles_dir)
    : cache_dir_(std::move(cache_dir)),
      local_tiles_dir_(std::move(local_tiles_dir)),
      timeout_sec_(timeout_sec)
{
    fs::create_directories(cache_dir_);
}

cv::Mat TileFetcher::fetch_tile(int x, int y, int zoom)
{
    // 1순위: 로컬 타일 디렉토리 (bing_{zoom}_{x}_{y}.jpg)
    if (!local_tiles_dir_.empty()) {
        std::string fname = "bing_" + std::to_string(zoom) + "_"
                          + std::to_string(x) + "_"
                          + std::to_string(y) + ".jpg";
        fs::path local_path = fs::path(local_tiles_dir_) / fname;
        if (fs::exists(local_path)) {
            cv::Mat img = cv::imread(local_path.string());
            if (!img.empty()) return img;
        }
    }

    // 2순위: 다운로드 캐시
    fs::path path = fs::path(cache_dir_)
        / std::to_string(zoom) / std::to_string(x)
        / (std::to_string(y) + ".jpg");
    fs::create_directories(path.parent_path());

    if (fs::exists(path)) {
        cv::Mat img = cv::imread(path.string());
        if (!img.empty()) return img;
        fs::remove(path);
    }

    // Esri World Imagery  {z}/{y}/{x}
    std::string url =
        "https://server.arcgisonline.com/ArcGIS/rest/services/"
        "World_Imagery/MapServer/tile/"
        + std::to_string(zoom) + "/"
        + std::to_string(y)    + "/"
        + std::to_string(x);

    // 최대 2회 재시도
    for (int attempt = 0; attempt < 2; ++attempt) {
        CURL* curl = curl_easy_init();
        if (!curl) break;

        std::string buf;
        curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  write_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &buf);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT,        static_cast<long>(timeout_sec_));
        curl_easy_setopt(curl, CURLOPT_USERAGENT,
            "Mozilla/5.0 (compatible; bootcamp-ros2/0.1)");
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK || buf.empty()) continue;

        std::vector<uchar> data(buf.begin(), buf.end());
        cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);
        if (img.empty()) continue;

        // 정상 타일만 캐시 저장
        cv::imwrite(path.string(), img, {cv::IMWRITE_JPEG_QUALITY, 95});
        return img;
    }

    // 다운로드 실패 → 완전 투명(검정)으로 처리해 회색 블록 방지
    return cv::Mat(TILE_SZ, TILE_SZ, CV_8UC3, cv::Scalar(0, 0, 0));
}

TileCanvas TileFetcher::build_canvas(double lat, double lon, int zoom, int grid)
{
    auto [cx, cy] = lat_lon_to_tile(lat, lon, zoom);
    int half = grid / 2, ox = cx - half, oy = cy - half;
    int sz   = grid * TILE_SZ;

    // 모든 타일 병렬 다운로드
    std::vector<std::future<cv::Mat>> futures;
    futures.reserve(grid * grid);
    for (int dy = 0; dy < grid; ++dy)
        for (int dx = 0; dx < grid; ++dx)
            futures.push_back(std::async(std::launch::async,
                [this, ox, oy, dx, dy, zoom]() {
                    return fetch_tile(ox + dx, oy + dy, zoom);
                }));

    // 캔버스: 완전 검정 배경 (실패 타일이 검정으로 보임 → 회색보다 깔끔)
    cv::Mat canvas(sz, sz, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int dy = 0; dy < grid; ++dy) {
        for (int dx = 0; dx < grid; ++dx) {
            cv::Mat t = futures[dy * grid + dx].get();
            if (t.size() != cv::Size(TILE_SZ, TILE_SZ))
                cv::resize(t, t, {TILE_SZ, TILE_SZ}, 0, 0, cv::INTER_AREA);
            t.copyTo(canvas(cv::Rect(dx * TILE_SZ, dy * TILE_SZ, TILE_SZ, TILE_SZ)));
        }
    }

    // 타일 경계 블렌딩: 경계선 1px를 이웃과 평균내어 이음새를 부드럽게
    for (int i = 1; i < grid; ++i) {
        int px = i * TILE_SZ;
        // 수직 경계
        canvas.col(px - 1) = canvas.col(px - 1) * 0.5 + canvas.col(px) * 0.5;
        canvas.col(px)     = canvas.col(px - 1);
        // 수평 경계
        canvas.row(px - 1) = canvas.row(px - 1) * 0.5 + canvas.row(px) * 0.5;
        canvas.row(px)     = canvas.row(px - 1);
    }

    auto [top_lat, left_lon] = tile_to_lat_lon(ox, oy, zoom);
    return {canvas, top_lat, left_lon, meters_per_pixel(lat, zoom)};
}
