#ifndef __DRONESTATE_H__
#define __DRONESTATE_H__

#include <Eigen/Eigen>
#include <mavlink.h>

class DroneStateEncoder {
public:
    virtual uint64_t get_sim_time() = 0;
    virtual void set_vector_state(Eigen::VectorXd state) = 0;
    virtual Eigen::VectorXd get_vector_state() = 0;
    virtual void get_attitude(float* attitude) = 0; // <float(4)>
    virtual void get_rpy_speed(float* rpy) = 0; // <float(3)>
    virtual void get_lat_lon_alt(float* lat_lon_alt) = 0; // <float(3)>
    virtual void get_ground_speed(float* x_y_z) = 0; // <float(3)>
    virtual void get_acceleration(float* x_y_z) = 0; // <float(3)>
    
    mavlink_message_t hil_state_quaternion_msg(uint8_t system_id, uint8_t component_id) {
        mavlink_message_t msg;
        float attitude[4];
        float rpy_speed[3];
        float lat_lon_alt[3];
        float ground_speed[3];
        float acceleration[3];

        this->get_attitude((float*)&attitude);
        this->get_rpy_speed((float*)&rpy_speed);
        this->get_lat_lon_alt((float*)&lat_lon_alt);
        this->get_ground_speed((float*)&ground_speed);
        this->get_acceleration((float*)&acceleration);

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
            acceleration[2],
        );

        return msg;
    }

};

#endif // __DRONESTATE_H__