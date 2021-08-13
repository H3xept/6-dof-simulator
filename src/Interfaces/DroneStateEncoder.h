#ifndef __DRONESTATE_H__
#define __DRONESTATE_H__

#include <Eigen/Eigen>
#include <mavlink.h>

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
        const float* euler = (const float*) state.data() + 9;
        for (uint i = 0; i < 3; i++) *(rpy+i) = *(euler+i);
    } 
    
    /**
     * Ground speed (lat. , lon. , alt.) in cm/s
     */
    void get_ground_speed(int16_t* x_y_z) { // <int16_t(3)>
        Eigen::VectorXd state_derivative = this->get_vector_dx_state();
        const int16_t* ground_speed = (const int16_t*) state_derivative.data();
        for (uint i = 0; i < 3; i++) *(x_y_z+i) = *(ground_speed+i);
    } 
    
    /**
     * Acceleration (ẍ , ÿ , z̈) in mG
     */
    void get_acceleration(int16_t* x_y_z) { // <int16_t(3)>
        Eigen::VectorXd state_derivative = this->get_vector_dx_state();
        const int16_t* acceleration = (const int16_t*) state_derivative.data();
        for (uint i = 0; i < 3; i++) *(x_y_z+i) = *(acceleration+i);
    } 

    void get_lat_lon_alt(float* lat_lon_alt) {  // <float(3)>
        Eigen::VectorXd state_derivative = this->get_vector_dx_state();
        const float* _lat_lon_alt = (const float*) state_derivative.data();
        for (uint i = 0; i < 3; i++) *(lat_lon_alt+i) = *(_lat_lon_alt+i);
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
        float attitude[4];
        float rpy_speed[3];
        float lat_lon_alt[3];
        int16_t ground_speed[3];
        int16_t acceleration[3];

        this->get_attitude((float*)&attitude);
        this->get_rpy_speed((float*)&rpy_speed);
        this->get_lat_lon_alt((float*)&lat_lon_alt);
        this->get_ground_speed((int16_t*)&ground_speed);
        this->get_acceleration((int16_t*)&acceleration);

        mavlink_msg_hil_state_quaternion_pack(
            system_id,
            component_id,
            &msg,
            this->get_sim_time(),
            (float*)&attitude,
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

};

#endif // __DRONESTATE_H__