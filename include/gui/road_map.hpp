#pragma once
#include <vector>
#include <string>
#include <utility>

class RoadMap {
public:
    using Polyline = std::vector<std::pair<double, double>>;

    RoadMap();
    bool loadFromJson(const std::string& filename);
    const std::vector<Polyline>& getPolylines() const;
    // Now takes origin UTM easting/northing to convert UTM to local XY
    void convertAllToLocalXY(double utm_e0, double utm_n0);

private:
    std::vector<Polyline> polylines_;
};