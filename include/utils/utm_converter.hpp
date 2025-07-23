#pragma once
#include <tuple>
#include <string>
#include <utility>

/**
 * @file utm_converter.hpp
 * @brief Utilities for converting between geographic (lat/lon/alt) and UTM coordinates.
 */

namespace utils {

/**
 * @class UTMConverter
 * @brief Utility class for converting between geodetic (latitude, longitude, altitude) and UTM coordinates.
 *
 * Provides static methods for common conversions, including local XY frame computation.
 */
class UTMConverter {
public:
    /**
     * @brief Convert geodetic coordinates to UTM.
     * @param latitude  Latitude in degrees.
     * @param longitude Longitude in degrees.
     * @param altitude  Altitude in meters.
     * @return Tuple (easting, northing, zone, zoneLetter, altitude).
     */
    static std::tuple<double, double, int, char, double>
    geodeticToUTM(double latitude, double longitude, double altitude);

    /**
     * @brief Convert UTM coordinates to geodetic.
     * @param easting    UTM easting (meters).
     * @param northing   UTM northing (meters).
     * @param zone       UTM zone number.
     * @param zoneLetter UTM zone letter.
     * @param altitude   Altitude in meters.
     * @return Tuple (latitude, longitude, altitude).
     */
    static std::tuple<double, double, double>
    UTMToGeodetic(double easting, double northing, int zone, char zoneLetter, double altitude);

    /**
     * @brief Get UTM zone letter from latitude.
     * @param latitude Latitude in degrees.
     * @return UTM zone letter.
     */
    static char latitudeToZoneLetter(double latitude);

    /**
     * @brief Get string representation of a UTM zone.
     * @param zone   UTM zone number.
     * @param letter UTM zone letter.
     * @return String in the form "33T", etc.
     */
    static std::string zoneString(int zone, char letter);

    /**
     * @brief Convert latitude/longitude to local XY coordinates relative to a given origin.
     * @param latitude  Latitude of point (degrees).
     * @param longitude Longitude of point (degrees).
     * @param lat0      Latitude of origin (degrees).
     * @param lon0      Longitude of origin (degrees).
     * @return Pair (x, y) in meters relative to (lat0, lon0).
     */
    static std::pair<double, double> latlonToLocalXY(double latitude, double longitude, double lat0, double lon0);

    static void latlonToUTM(double latitude, double longitude, double& easting, double& northing, int& zone, bool& northp) {
        double alt;
        char zoneLetter;
        std::tie(easting, northing, zone, zoneLetter, alt) = geodeticToUTM(latitude, longitude, 0.0);
        northp = (zoneLetter >= 'N');
    }
};

} // namespace utils