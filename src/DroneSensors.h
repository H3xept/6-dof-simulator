#ifndef __DRONESENSORS_H__
#define __DRONESENSORS_H__

#include "Interfaces/Sensors.h"
#include "Interfaces/DynamicObject.h"
#include "Interfaces/TimeHandler.h"

class DroneSensors : public Sensors {
private:
    uint32_t update_step = 0;
    // Send gps update every n simulation updates
    uint8_t gps_update_throttle = 2;

    // Standard deviations for sensor noise
    float noise_Acc = 0.005f;
    float noise_Gyo = 0.001f;
    float noise_Mag = 0.0005f;
    float noise_Prs = 0.0001f;

    std::default_random_engine noise_generator;
    double randomNoise(float stdDev) {
        std::normal_distribution<double> dist(0, stdDev);
        double n = dist(noise_generator);
        return n;
    }
    
protected:
    DynamicObject& drone;
    LatLonAlt gps_origin;
public:
    DroneSensors(DynamicObject& drone, LatLonAlt gps_origin);
    ~DroneSensors() {};
    Eigen::Vector3d get_body_frame_origin() override;
    Eigen::Vector3d get_body_frame_velocity() override;
    Eigen::Vector3d get_attitude() override;
    Eigen::Vector3d get_gyro() override;
    Eigen::Vector3d get_earth_frame_velocity() override;
    Eigen::Vector3d get_body_frame_acceleration() override;
    Eigen::Vector3d get_earth_frame_angle_rates() override;
    Eigen::Vector3d get_body_frame_angular_acceleration() override;
    Eigen::Vector3d get_magnetic_field(LatLonAlt lat_lon_alt) override;
    double get_pressure() override;
    GPSData get_gps_data() override;
    LatLonAlt get_lat_lon_alt() override;
    uint16_t get_yaw_wrt_earth_north() override;
    uint16_t get_course_over_ground() override;
    bool new_gps_data() override;
    Eigen::Vector3d get_environment_wind() override;
    uint16_t get_true_wind_speed() override;
    Eigen::Vector3d get_absolute_ground_speed() override;
    double get_environment_temperature() override;
};

#endif // __DRONESENSORS_H__