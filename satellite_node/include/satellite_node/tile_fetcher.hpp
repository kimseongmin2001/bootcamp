#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>

struct TileCanvas {
    cv::Mat image;
    double  top_lat, left_lon, mpp;
};

std::pair<int,int>       lat_lon_to_tile(double lat, double lon, int zoom);
std::pair<double,double> tile_to_lat_lon(int x, int y, int zoom);
double                   meters_per_pixel(double lat, int zoom);

class TileFetcher {
public:
    explicit TileFetcher(std::string cache_dir, double timeout_sec = 5.0);
    TileCanvas build_canvas(double lat, double lon, int zoom, int grid);
private:
    cv::Mat fetch_tile(int x, int y, int zoom);
    std::string cache_dir_;
    double      timeout_sec_;
};
