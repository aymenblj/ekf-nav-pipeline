#include "utils/utm_converter.hpp"
#include <cmath>
#include <stdexcept>
#include <sstream>

namespace utils {

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_ECCSQ = 0.00669437999014;
constexpr double M_PI = 3.14159265358979323846;
constexpr double K0 = 0.9996;

static double deg2rad(double deg) { return deg * M_PI / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

char UTMConverter::latitudeToZoneLetter(double latitude) {
    if (latitude < -80) return 'C';
    if (latitude < -72) return 'C';
    if (latitude < -64) return 'D';
    if (latitude < -56) return 'E';
    if (latitude < -48) return 'F';
    if (latitude < -40) return 'G';
    if (latitude < -32) return 'H';
    if (latitude < -24) return 'J';
    if (latitude < -16) return 'K';
    if (latitude < -8)  return 'L';
    if (latitude < 0)   return 'M';
    if (latitude < 8)   return 'N';
    if (latitude < 16)  return 'P';
    if (latitude < 24)  return 'Q';
    if (latitude < 32)  return 'R';
    if (latitude < 40)  return 'S';
    if (latitude < 48)  return 'T';
    if (latitude < 56)  return 'U';
    if (latitude < 64)  return 'V';
    if (latitude < 72)  return 'W';
    if (latitude < 84)  return 'X';
    return 'X';
}

std::string UTMConverter::zoneString(int zone, char letter) {
    std::ostringstream oss;
    oss << zone << letter;
    return oss.str();
}

std::tuple<double, double, int, char, double>
UTMConverter::geodeticToUTM(double latitude, double longitude, double altitude) {
    int zone = int((longitude + 180) / 6) + 1;
    char zoneLetter = latitudeToZoneLetter(latitude);

    double latRad = deg2rad(latitude);
    double lonRad = deg2rad(longitude);
    double lonOrigin = (zone - 1) * 6 - 180 + 3;
    double lonOriginRad = deg2rad(lonOrigin);

    double eccPrimeSquared = WGS84_ECCSQ / (1 - WGS84_ECCSQ);

    double N = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(latRad) * sin(latRad));
    double T = tan(latRad) * tan(latRad);
    double C = eccPrimeSquared * cos(latRad) * cos(latRad);
    double A = cos(latRad) * (lonRad - lonOriginRad);

    double M = WGS84_A * (
        (1 - WGS84_ECCSQ / 4 - 3 * WGS84_ECCSQ * WGS84_ECCSQ / 64 - 5 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256) * latRad
        - (3 * WGS84_ECCSQ / 8 + 3 * WGS84_ECCSQ * WGS84_ECCSQ / 32 + 45 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024) * sin(2 * latRad)
        + (15 * WGS84_ECCSQ * WGS84_ECCSQ / 256 + 45 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024) * sin(4 * latRad)
        - (35 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072) * sin(6 * latRad)
    );

    double easting = K0 * N * (A + (1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120) + 500000.0;
    double northing = K0 * (M + N * tan(latRad) * (A*A/2 + (5-T+9*C+4*C*C)*A*A*A*A/24
                        + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720));
    if (latitude < 0)
        northing += 10000000.0;

    return std::make_tuple(easting, northing, zone, zoneLetter, altitude);
}

std::tuple<double, double, double>
UTMConverter::UTMToGeodetic(double easting, double northing, int zone, char zoneLetter, double altitude) {
    double x = easting - 500000.0;
    double y = northing;
    if (zoneLetter < 'N') y -= 10000000.0;

    double lonOrigin = (zone - 1) * 6 - 180 + 3;
    double eccPrimeSquared = WGS84_ECCSQ / (1 - WGS84_ECCSQ);

    double M = y / K0;
    double mu = M / (WGS84_A * (1 - WGS84_ECCSQ / 4 - 3 * WGS84_ECCSQ * WGS84_ECCSQ / 64 - 5 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256));

    double e1 = (1 - sqrt(1 - WGS84_ECCSQ)) / (1 + sqrt(1 - WGS84_ECCSQ));

    double J1 = (3*e1/2 - 27*e1*e1*e1/32);
    double J2 = (21*e1*e1/16 - 55*e1*e1*e1*e1/32);
    double J3 = (151*e1*e1*e1/96);
    double J4 = (1097*e1*e1*e1*e1/512);

    double fp = mu + J1*sin(2*mu) + J2*sin(4*mu) + J3*sin(6*mu) + J4*sin(8*mu);

    double sinfp = sin(fp);
    double cosfp = cos(fp);

    double C1 = eccPrimeSquared * cosfp * cosfp;
    double T1 = tan(fp) * tan(fp);
    double N1 = WGS84_A / sqrt(1 - WGS84_ECCSQ * sinfp * sinfp);
    double R1 = WGS84_A * (1 - WGS84_ECCSQ) / pow(1 - WGS84_ECCSQ * sinfp * sinfp, 1.5);
    double D = x / (N1 * K0);

    double lat = fp - (N1 * tan(fp) / R1) * (
        D*D/2 - (5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
        + (61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720
    );
    double lon = deg2rad(lonOrigin) + (
        D - (1+2*T1+C1)*D*D*D/6 + (5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)*D*D*D*D*D/120
    ) / cosfp;

    return std::make_tuple(rad2deg(lat), rad2deg(lon), altitude);
}

std::pair<double, double> UTMConverter::latlonToLocalXY(double latitude, double longitude, double lat0, double lon0) {
    double x, y, x0, y0;
    std::tie(x, y, std::ignore, std::ignore, std::ignore) = geodeticToUTM(latitude, longitude, 0.0);
    std::tie(x0, y0, std::ignore, std::ignore, std::ignore) = geodeticToUTM(lat0, lon0, 0.0);
    return {x - x0, y - y0};
}
}