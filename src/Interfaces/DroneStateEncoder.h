#ifndef __DRONESTATE_H__
#define __DRONESTATE_H__

#include "../Helpers/magnetic_field_lookup.h"
#include <cmath>
#include <boost/chrono.hpp>
#include <Eigen/Eigen>
#include <mavlink.h>
#include <EquationsOfMotion/rotationMatrix.h>

class DroneStateEncoder {
protected:

    void euler_to_quaterions(const float* euler_rpy, float* quaternion) {
        float roll = euler_rpy[0];
        float pitch = euler_rpy[1];
        float yaw = euler_rpy[2];
        quaternion[0] = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
        quaternion[1] = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
        quaternion[2] = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
        quaternion[3] = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    }

    void get_attitude(float* attitude) { // <float(4)>
        Eigen::VectorXd& state = this->get_vector_state();
        const float* euler = (const float*) state.data() + 6;
        this->euler_to_quaterions(euler, attitude);
    } 

    void get_rpy_speed(float* rpy) { // <float(3)>
        Eigen::VectorXd state = this->get_vector_state();
        for (uint i = 0; i < 3; i++) *(rpy+i) = state[i + 6];
    } 
    
    /**
     * Ground speed (lat. , lon. , alt.) in cm/s
     */
    void get_ground_speed(int16_t* x_y_z) { // <int16_t(3)>
        Eigen::VectorXd state_derivative = this->get_vector_dx_state();
        for (uint i = 0; i < 3; i++) x_y_z[i] = state_derivative[i];
        x_y_z[0] *= 100;
        x_y_z[1] *= 100;
        x_y_z[2] *= 100;
    } 
    
    /**
     * Body frame (NED) acceleration (ẍ , ÿ , z̈) in mG
     */
    void get_body_frame_acceleration(float* x_y_z) { // <float(3)>
#define G_FORCE 9.81
        Eigen::VectorXd state_derivative = this->get_vector_dx_state();
        for (uint i = 0; i < 3; i++) *(x_y_z+i) = state_derivative[3 + i];
    } 

    /**
     * Body frame origin (x,y,z) in NED with respect to earth frame
     */
    void get_body_frame_origin(float* x_y_z) { // <float(3)>
        Eigen::VectorXd state = this->get_vector_state();
        for (uint i = 0; i < 3; i++) *(x_y_z+i) = state[i];
    }
    
    /**
     * lat: [degE7]
     * lon: [degE7]
     * alt: [mm]
     */
    void get_lat_lon_alt(int32_t* lat_lon_alt) {  // <int32_t(3)>
// UK Grid origin
#define INITIAL_LAT 49.766809
#define INITIAL_LON -7.5571598
        Eigen::VectorXd state = this->get_vector_state();
        double d_lat_lon_alt[3] = {0};
        caelus_fdm::convertState2LlA(INITIAL_LAT, INITIAL_LON, state, d_lat_lon_alt[0], d_lat_lon_alt[1], d_lat_lon_alt[2]);
        lat_lon_alt[0] = (int32_t)(d_lat_lon_alt[0]);
        lat_lon_alt[1] = (int32_t)(d_lat_lon_alt[1]);
        lat_lon_alt[2] = (int32_t)((d_lat_lon_alt[2] * 1000)); // m to mm
    }
    
public:
    virtual uint64_t get_sim_time() = 0;

    /**
     * Drone state as populated by the CAELUS_FDM package.
     * <
     *  x , y , z    [0:3]  body-frame origin with respect to earth-frame (NED m)
     *  ẋ , ẏ , ż    [3:6]  body-frame velocity (m/s)
     *  ɸ , θ , ѱ    [6:9]  (roll, pitch, yaw) body-frame orientation with respect to earth-frame (rad/s)
     *  ɸ. , θ. , ѱ. [9:12] (roll., pitch., yaw.) body-frame orientation velocity (rad/s**2)
     * >
     */
    virtual Eigen::VectorXd& get_vector_state() = 0;
    /**
     *  (FixedWingEOM.h:evaluate)
     *  Drone state derivative as populated by the CAELUS_FDM package.
     *  ẋ , ẏ , ż       [0:3]  earth-frame velocity (What unit?)
     *  ẍ , ÿ , z̈       [3:6]  body-frame acceleration (m/s**2)
     *  ? , ? , ?       [6:9]  earth-frame angle rates (Euler rates?)
     *  ɸ.. , θ.. , ѱ.. [9:12] body-frame angular acceleration (What unit?)
     */
    virtual Eigen::VectorXd& get_vector_dx_state() = 0;
    
    mavlink_message_t hil_state_quaternion_msg(uint8_t system_id, uint8_t component_id) {
        mavlink_message_t msg;
        float attitude[4] = {0};
        float rpy_speed[3] = {0};
        int32_t lat_lon_alt[3] = {0};
        int16_t ground_speed[3] = {0};
        float f_acceleration[3] = {0};
        int16_t acceleration[3] = {0};

        this->get_attitude((float*)attitude);
        this->get_rpy_speed((float*)rpy_speed);
        this->get_lat_lon_alt((int32_t*)lat_lon_alt);
        this->get_ground_speed((int16_t*)ground_speed);

        this->get_body_frame_acceleration((float*)f_acceleration);
        // m/s**2 to mG (milli Gs)
        acceleration[0] = (int16_t)std::round((f_acceleration[0] / G_FORCE) * 1000);
        acceleration[1] = (int16_t)std::round((f_acceleration[1] / G_FORCE) * 1000);
        acceleration[2] = (int16_t)std::round((f_acceleration[2] / G_FORCE) * 1000);

        // Vector3d airSpeed = new Vector3d(vehicle.getVelocity());
        // airSpeed.scale(-1.0);
        // airSpeed.add(vehicle.getWorld().getEnvironment().getCurrentWind(vehicle.position));
        // float as_mag = (float) airSpeed.length();
        // msg_hil_state.set("true_airspeed", (int)(as_mag * 100));

        mavlink_msg_hil_state_quaternion_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            (float*)attitude,
            rpy_speed[0],
            rpy_speed[1],
            rpy_speed[2],
            lat_lon_alt[0],
            lat_lon_alt[1],
            lat_lon_alt[2],
            ground_speed[0],
            ground_speed[1],
            ground_speed[2],
            0,
            0,
            acceleration[0],
            acceleration[1],
            acceleration[2]
        );

        return msg;
    }

    mavlink_message_t hil_sensor_msg(uint8_t system_id, uint8_t component_id) {
        mavlink_message_t msg;
        
        int32_t lat_lon_alt[3] = {0};
        float body_frame_acc[3] = {0}; // m/s**2
        float gyro_xyz[3] = {0}; // rad/s
        float magfield[3] = {0}; // gauss
        float abs_pressure = 1033.0; // hPa // Random value from (http://www.isleofskyeweather.co.uk/wxbarosummary.php)
        float diff_pressure = abs_pressure;
        float pressure_alt = 100; // Fixed value
        float temperature = 25; // degC

        this->get_body_frame_acceleration((float*)body_frame_acc);
        this->get_rpy_speed((float*) gyro_xyz);
        this->get_lat_lon_alt((int32_t*)lat_lon_alt);

        Eigen::VectorXd mag_field_vec = magnetic_field_for_latlonalt((int32_t*)lat_lon_alt);
        for (int i = 0; i < mag_field_vec.size(); i++) magfield[i] = mag_field_vec[i];

        mavlink_msg_hil_sensor_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            body_frame_acc[0],
            body_frame_acc[1],
            body_frame_acc[2],
            gyro_xyz[0],
            gyro_xyz[1],
            gyro_xyz[2],
            magfield[0],
            magfield[1],
            magfield[2],
            abs_pressure,
            0.0,
            488.17853,
            0,
            4095,
            0
        );

        return msg;
    }

    mavlink_message_t hil_gps_msg(uint8_t system_id, uint8_t component_id) {
        mavlink_message_t msg;
        
        // DegE7, DegE7, mm
        float lon_lat_alt[3] = {0};
        // Earth-fixed NED frame (cm/s)
        uint16_t gps_ground_speed = UINT16_MAX; // uint16 max for not available
        // cm/s
        int16_t gps_velocity_ned[3] = {0};
        // Course Over Ground is the actual direction of progress of a vessel, 
        // between two points, with respect to the surface of the earth.
        uint16_t course_over_ground = 20; // 0.0..359.99 degrees // cdeg
        // number of visible satellites
        uint8_t sat_visible = UINT8_MAX;
        // Relative to earth's north
        uint8_t vehicle_yaw = 0; // 0 means not available


        // this->get_lat_lon_alt((float*)&lon_lat_alt);

        mavlink_msg_hil_gps_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            3, // 3d fix
            lon_lat_alt[1],
            lon_lat_alt[0],
            lon_lat_alt[2],
            UINT16_MAX,
            UINT16_MAX,
            gps_ground_speed,
            gps_velocity_ned[0],
            gps_velocity_ned[1],
            gps_velocity_ned[2],
            course_over_ground,
            sat_visible,
            0,
            vehicle_yaw
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