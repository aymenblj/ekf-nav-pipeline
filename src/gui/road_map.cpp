#include "gui/road_map.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include "utils/utm_converter.hpp"

RoadMap::RoadMap() {}

bool RoadMap::loadFromJson(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open())
        return false;
    nlohmann::json j;
    file >> j;
    polylines_.clear();
    for (const auto& arr : j) {
        Polyline poly;
        for (const auto& pt : arr) {
            if (pt.size() != 2) continue;
            poly.emplace_back(pt[0].get<double>(), pt[1].get<double>());
        }
        if (!poly.empty())
            polylines_.push_back(std::move(poly));
    }
    return true;
}

void RoadMap::convertAllToLocalXY(double lat0, double lon0) {
    for (auto& poly : polylines_) {
        for (auto& pt : poly) {
            // pt = {lon, lat}. Conversion requires (lat, lon)
            auto [x, y] = utils::UTMConverter::latlonToLocalXY(pt.second, pt.first, lat0, lon0);
            pt.first = x;
            pt.second = y;
        }
    }
}

const std::vector<RoadMap::Polyline>& RoadMap::getPolylines() const {
    return polylines_;
}