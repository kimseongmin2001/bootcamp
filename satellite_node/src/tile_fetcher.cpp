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

TileFetcher::TileFetcher(std::string cache_dir, double timeout_sec)
    : cache_dir_(std::move(cache_dir)), timeout_sec_(timeout_sec)
{
    fs::create_directories(cache_dir_);
}

cv::Mat TileFetcher::fetch_tile(int x, int y, int zoom)
{
    // Esri 위성 이미지는 JPEG 반환 → 캐시도 .jpg
    fs::path path = fs::path(cache_dir_)
        / std::to_string(zoom) / std::to_string(x) / (std::to_string(y) + ".jpg");
    fs::create_directories(path.parent_path());

    if (fs::exists(path)) {
        cv::Mat img = cv::imread(path.string());
        if (!img.empty()) return img;
    }

    // Esri World Imagery — 무료 위성 타일 (API 키 불필요)
    // 주의: Esri 타일 URL은 {z}/{y}/{x} 순서 (OSM과 다르게 y가 앞)
    std::string url = "https://server.arcgisonline.com/ArcGIS/rest/services/"
        "World_Imagery/MapServer/tile/"
        + std::to_string(zoom) + "/" + std::to_string(y)
        + "/" + std::to_string(x);

    CURL* curl = curl_easy_init();
    if (!curl) return cv::Mat(TILE_SZ, TILE_SZ, CV_8UC3, cv::Scalar(50,50,50));

    std::string buf;
    curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &buf);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        static_cast<long>(timeout_sec_));
    curl_easy_setopt(curl, CURLOPT_USERAGENT,
        "Mozilla/5.0 (compatible; bootcamp-ros2/0.1)");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK || buf.empty())
        return cv::Mat(TILE_SZ, TILE_SZ, CV_8UC3, cv::Scalar(50,50,50));

    std::vector<uchar> data(buf.begin(), buf.end());
    cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);
    if (img.empty()) return cv::Mat(TILE_SZ, TILE_SZ, CV_8UC3, cv::Scalar(50,50,50));

    cv::imwrite(path.string(), img,
        {cv::IMWRITE_JPEG_QUALITY, 95});
    return img;
}

TileCanvas TileFetcher::build_canvas(double lat, double lon, int zoom, int grid)
{
    auto [cx, cy] = lat_lon_to_tile(lat, lon, zoom);
    int half = grid/2, ox = cx-half, oy = cy-half;
    int sz   = grid * TILE_SZ;

    // 병렬 다운로드: 모든 타일을 async로 동시에 요청
    int total = grid * grid;
    std::vector<std::future<cv::Mat>> futures;
    futures.reserve(total);
    for (int dy = 0; dy < grid; ++dy)
        for (int dx = 0; dx < grid; ++dx)
            futures.push_back(std::async(std::launch::async,
                [this, ox, oy, dx, dy, zoom]() {
                    return fetch_tile(ox+dx, oy+dy, zoom);
                }));

    cv::Mat canvas(sz, sz, CV_8UC3, cv::Scalar(40,40,40));
    for (int dy = 0; dy < grid; ++dy)
        for (int dx = 0; dx < grid; ++dx) {
            cv::Mat t = futures[dy*grid+dx].get();
            cv::resize(t, t, {TILE_SZ, TILE_SZ});
            t.copyTo(canvas(cv::Rect(dx*TILE_SZ, dy*TILE_SZ, TILE_SZ, TILE_SZ)));
        }

    auto [top_lat, left_lon] = tile_to_lat_lon(ox, oy, zoom);
    return {canvas, top_lat, left_lon, meters_per_pixel(lat, zoom)};
}
