#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include "../Logging/ConsoleLogger.h"
#include "http_req.h"
#include "magnetic_field_lookup.h"

#define MAG_LOOKUP_ENDPOINT "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?resultFormat=csv&coordinateSystem=M&lat1=%1%&lon1=%2%&alt=%3%"
#define TO_RAD(d) ((d * M_PI) / 180)

Eigen::VectorXd extract_mag_field(double mag_inclination_deg, double mag_declination_deg) {
    Eigen::VectorXd mag_field {3};
    mag_field[0] = cos(TO_RAD(mag_inclination_deg));
    mag_field[1] = 0.0;
    mag_field[2] = sin(TO_RAD(mag_declination_deg));
    return mag_field;
}

std::vector<double> parse_doubles_in_line(std::string s) {
    std::vector<std::string> values_s;
    std::vector<double> values;
    boost::split(values_s, s, [](char c){return c == ',';});    
    for (int i = 0; i < values_s.size(); i++) {
        values.push_back(std::stod(values_s[i]));
    }
    return values;
}

Eigen::VectorXd parse_response_to_vector(std::string s) {

    std::vector<std::string> lines;
    boost::split(lines, s, [](char c){return c == '\n';});
    // Last line of the canonical response from MAG_LOOKUP_ENDPOINT
    std::vector<double> mag_values = parse_doubles_in_line(lines[lines.size() - 2]);

    // Indices are specified in the response (which is human readable)
    double inc_deg = mag_values[1];
    double dec_deg = mag_values[2];

    return extract_mag_field(inc_deg, dec_deg);
}

static Eigen::VectorXd cached_magfield = Eigen::VectorXd::Zero(3);

Eigen::VectorXd magnetic_field_for_latlonalt(const int32_t* lat_lon_alt) {
    // printf("Lat lon alt: %d %d %d\n", lat_lon_alt[0], lat_lon_alt[1], lat_lon_alt[2]);
    try {
        if (cached_magfield.isZero()) {
            URL u(boost::str((boost::format(MAG_LOOKUP_ENDPOINT) % 
                    (lat_lon_alt[0] / 1.e7) % // LatLon are converted to int form by multiplying by 1.e7 in DroneStateEncode 
                    (lat_lon_alt[1] / 1.e7) % // ^^^^^^^^^^ This is a re-normalization to decimal form
                    ((lat_lon_alt[2] / 1000) / 1000) // mm to km
                )));
            cached_magfield = parse_response_to_vector(u.get_body());
        } return cached_magfield;
    } catch(const std::exception& e) {
        ConsoleLogger* logger = ConsoleLogger::shared_instance();
        logger->debug_log("Error in magnetic_field_for_latlonalt");
        fprintf(stderr, "%s\n", e.what());
    }
}