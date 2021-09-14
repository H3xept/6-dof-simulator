#ifndef __DRONESTATE_H__
#define __DRONESTATE_H__

#include "../Helpers/magnetic_field_lookup.h"
#include "../Logging/ConsoleLogger.h"
#include "Sensors.h"
#include "../Helpers/rotation_utils.h"
#include "../Helpers/rotationMatrix.h"
#include "../DataStructures/LatLonAlt.h"
#include "../DataStructures/GPSData.h"
#include "../DataStructures/GroundSpeed.h"

#include <cmath>
#include <boost/chrono.hpp>
#include <Eigen/Eigen>
#include <mavlink.h>

// #define HIL_STATE_QUATERNION_VERBOSE
// #define HIL_SENSOR_VERBOSE
// #define HIL_GPS_VERBOSE
#define SENSOR_RANDOM_NOISE

class DroneStateEncoder {
private:
    // Standard deviations for sensor noise
    float noise_Acc = 0.005f;
    float noise_Gyo = 0.001f;
    float noise_Mag = 0.0005f;
    float noise_Prs = 0.0001f;

    std::default_random_engine noise_generator;
    double randomNoise(float stdDev) {
#ifdef SENSOR_RANDOM_NOISE
        std::normal_distribution<double> dist(0, stdDev);
        double n = dist(noise_generator);
        return n;
#else
        return 0.0;
#endif
    }

    mavlink_message_t _hil_sensor_msg(
        uint8_t system_id,
        uint8_t component_id,
        float x_acc,
        float y_acc,
        float z_acc,
        float x_gyro,
        float y_gyro,
        float z_gyro,
        float x_mag,
        float y_mag,
        float z_mag,
        float abs_pressure,
        float diff_pressure,
        float pressure_alt,
        float temperature,
        uint32_t fields_updated = 0b1101111111111
    )
    {
        mavlink_message_t msg;

#ifdef HIL_SENSOR_VERBOSE
        printf("[HIL_SENSOR]\n");
        printf("Body frame Acceleration (m/s**2): %f %f %f \n", x_acc, y_acc, z_acc);
        printf("GYRO xyz speed (rad/s): %f %f %f \n", x_gyro, y_gyro, z_gyro);
        printf("Magfield (gauss): %f %f %f \n", x_mag, y_mag, z_mag);
        printf("Absolute pressure (hPa): %f\n", abs_pressure);
        printf("Differential pressure (hPa): %f\n", diff_pressure);
        printf("Alt (m) : %f \n", pressure_alt);
        printf("Temperature (C): %f\n", temperature);
        printf("Sensor bitfield: %d\n", fields_updated);
        printf("Sim time %llu\n", this->get_sim_time());
#endif

        mavlink_msg_hil_sensor_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            x_acc,
            y_acc,
            z_acc,
            x_gyro,
            y_gyro,
            z_gyro,
            x_mag,
            y_mag,
            z_mag,
            abs_pressure,
            diff_pressure,
            pressure_alt,
            temperature,
            fields_updated,
            0 // ID
        );

        return msg;
    }

    mavlink_message_t _hil_state_quaternion_msg(
        uint8_t system_id,
        uint8_t component_id,
        float* attitude_quaternion,
        float roll_speed,
        float pitch_speed,
        float yaw_speed,
        int32_t lat,
        int32_t lon, 
        int32_t alt,
        int16_t vx,
        int16_t vy,
        int16_t vz,
        uint16_t indicated_airspeed,
        uint16_t true_airspeed,
        int16_t x_acc,
        int16_t y_acc,
        int16_t z_acc) 
    {
        mavlink_message_t msg;
#ifdef HIL_STATE_QUATERNION_VERBOSE
        Eigen::VectorXd attitude_euler = this->get_sensors().get_earth_frame_attitude();

        printf("[HIL STATE QUATERNION]\n");
        printf("Attitude quaternion: %f %f %f %f \n", attitude_quaternion[0], attitude_quaternion[1], attitude_quaternion[2], attitude_quaternion[3]);
        printf("Attitude euler (rad): roll: %f pitch: %f yaw: %f \n", attitude_euler[0], attitude_euler[1], attitude_euler[2]);
        printf("RPY Speed (rad/s): %f %f %f \n", roll_speed, pitch_speed, yaw_speed);
        printf("Lat Lon Alt (degE7, degE7, mm): %d %d %d \n", lat, lon, alt);
        printf("Ground speed (cm/s): %d %d %d \n", vx, vy, vz);
        printf("Acceleration (mG): %hd %hd %hd \n", x_acc, y_acc, z_acc);
        printf("True air speed (cm/s): %d \n", true_airspeed);
        printf("Indicated air speed (cm/s): %d \n", indicated_airspeed);
        printf("Sim time %llu\n", this->get_sim_time());
#endif

        mavlink_msg_hil_state_quaternion_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            attitude_quaternion,
            roll_speed,
            pitch_speed,
            yaw_speed,
            lat,
            lon,
            alt,
            vx,
            vy,
            vz,
            indicated_airspeed,
            true_airspeed,
            x_acc,
            y_acc,
            z_acc
        );
        
        return msg;
    }

protected:

public:
    virtual uint64_t get_sim_time() = 0;
    virtual Sensors& get_sensors() = 0;
    
    mavlink_message_t hil_state_quaternion_msg(uint8_t system_id, uint8_t component_id) {
        Sensors& sensors = this->get_sensors();

        Eigen::VectorXd attitude = euler_angles_to_quaternions(sensors.get_earth_frame_attitude());
        float attitude_float[4] = {0};
        for (int i = 0; i < attitude.size(); i++) attitude_float[i] = attitude[i];

        Eigen::Vector3d rpy_speed = sensors.get_body_frame_gyro();
        LatLonAlt lat_lon_alt = sensors.get_lat_lon_alt();
        Eigen::Vector3d ground_speed = sensors.get_absolute_ground_speed() * 100; // m to cm
        Eigen::Vector3d body_frame_acc = sensors.get_body_frame_acceleration();
        uint16_t true_wind_speed = sensors.get_true_wind_speed();

        // (acc / G * 1000) => m/s**2 to mG (milli Gs)
        body_frame_acc[0] = (int16_t)std::round((body_frame_acc[0] / fabs(G_FORCE)) * 1000);
        body_frame_acc[1] = (int16_t)std::round((body_frame_acc[1] / fabs(G_FORCE)) * 1000);
        body_frame_acc[2] = (int16_t)std::round((body_frame_acc[2] / fabs(G_FORCE)) * 1000);

        return this->_hil_state_quaternion_msg(
            system_id,
            component_id,
            attitude_float,
            rpy_speed[0],
            rpy_speed[1],
            rpy_speed[2],
            lat_lon_alt.latitude_deg * 1e7,
            lat_lon_alt.longitude_deg * 1e7,
            lat_lon_alt.altitude_mm,
            ground_speed[0],
            ground_speed[1],
            ground_speed[2],
            true_wind_speed,
            true_wind_speed,
            body_frame_acc[0],
            body_frame_acc[1],
            -body_frame_acc[2]
        );
    }

    mavlink_message_t hil_sensor_msg(uint8_t system_id, uint8_t component_id) {
        Sensors& sensors = this->get_sensors();
        
        LatLonAlt lat_lon_alt = sensors.get_lat_lon_alt();
        // m/s**2
        Eigen::Vector3d body_frame_acc = sensors.get_body_frame_acceleration();
        // rad/s
        Eigen::Vector3d gyro_xyz = sensors.get_body_frame_gyro();
        // gauss
        Eigen::Vector3d magfield = sensors.get_magnetic_field();
        double abs_pressure = sensors.get_pressure() / 100; // Pa to hPa
        double temperature = sensors.get_environment_temperature();
        float diff_pressure = 0;

        return this->_hil_sensor_msg(
            system_id,
            component_id, 
            body_frame_acc[0] + randomNoise(this->noise_Acc),
            body_frame_acc[1] + randomNoise(this->noise_Acc),
            -body_frame_acc[2] + randomNoise(this->noise_Acc),
            gyro_xyz[0] + randomNoise(this->noise_Gyo),
            gyro_xyz[1] + randomNoise(this->noise_Gyo),
            gyro_xyz[2] + randomNoise(this->noise_Gyo),
            magfield[0] + randomNoise(this->noise_Mag),
            magfield[1] + randomNoise(this->noise_Mag),
            magfield[2] + randomNoise(this->noise_Mag),
            abs_pressure + randomNoise(this->noise_Prs) / 100,
            diff_pressure + randomNoise(this->noise_Prs) / 100,
            // ENU to NED
            -lat_lon_alt.altitude_mm / 1000 + randomNoise(this->noise_Prs) , 
            temperature + randomNoise(this->noise_Prs)
        );
    }

    mavlink_message_t hil_gps_msg(uint8_t system_id, uint8_t component_id) {
        Sensors& sensors = this->get_sensors();
        mavlink_message_t msg;

        GPSData gps_data = sensors.get_gps_data();
        GroundSpeed gs = gps_data.ground_speed;
        LatLonAlt lat_lon_alt = gps_data.lat_lon_alt;

#ifdef HIL_GPS_VERBOSE
        printf("[GPS SENSOR]\n");
        printf("Lon Lat Alt (degE7, degE7, mm): %d %d %d \n", lat_lon_alt.latitude_deg, lat_lon_alt.longitude_deg, lat_lon_alt.altitude_mm);
        printf("EPH EPV (dimensionless): %d %d \n", gps_data.eph, gps_data.epv);
        printf("Ground speed (m/s): %d %d %d \n", gs.north_speed, gs.east_speed, gs.down_speed);
        printf("GPS ground speed (m/s): %d\n", gps_data.gps_ground_speed);
        printf("Course over ground: %d \n",  gps_data.course_over_ground);
        printf("Sats visible: %d \n",  gps_data.satellites_visible);
        printf("Vehicle yaw (deg): %d \n",  gps_data.vehicle_yaw);
        printf("Sim time %llu\n", this->get_sim_time());
#endif

        mavlink_msg_hil_gps_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            3, // 3d fix
            lat_lon_alt.latitude_deg * 1e7,
            lat_lon_alt.longitude_deg * 1e7,
            lat_lon_alt.altitude_mm,
            gps_data.eph,
            gps_data.epv,
            gps_data.gps_ground_speed,
            gs.north_speed,
            gs.east_speed,
            gs.down_speed,
            gps_data.course_over_ground,
            gps_data.satellites_visible,
            0, // ID
            gps_data.satellites_visible
        );

        return msg;
    }

    mavlink_message_t system_time_msg(uint8_t system_id, uint8_t component_id) {
        mavlink_message_t msg;

        uint64_t time_unix_usec = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::system_clock::now().time_since_epoch()).count();
        uint32_t time_boot_ms = this->get_sim_time() / 1000; // us to ms

        mavlink_msg_system_time_pack(
            system_id,
            component_id,
            &msg,
            time_unix_usec,
            time_boot_ms
        );

        return msg;
    }
};

#endif // __DRONESTATE_H__