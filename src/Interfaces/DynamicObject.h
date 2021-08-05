#ifndef __DYNAMICOBJECT_H__
#define __DYNAMICOBJECT_H__

#include <array>
#include "EnvironmentObject.h"

typedef std::array<double, 3> Vector3;
typedef Vector3 LatLonAlt;

struct Attitude {
    double yaw;
    double pitch;
    double roll;
};

typedef Attitude AttitudeDot;
typedef Attitude AttitudeDotDot;

struct DynamicObject : public EnvironmentObject {
public:
    ~DynamicObject() {};
    LatLonAlt position;
    Vector3 velocity;
    Vector3 acceleration;
    Attitude attitude;
    AttitudeDot angular_velocity;
    AttitudeDotDot angular_acceleration;
};

#endif // __DYNAMICOBJECT_H__