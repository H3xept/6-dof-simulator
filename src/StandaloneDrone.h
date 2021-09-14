#ifndef __STANDALONEDRONE_H__
#define __STANDALONEDRONE_H__

#include "Interfaces/DynamicObject.h"
#include "Propellers.h"
#include "Ailerons.h"
#include "DroneSensors.h"
#include "DataStructures/LatLonAlt.h"
#include "Containers/DroneConfig.h"

class StandaloneDrone : public DynamicObject {
protected:
    DroneConfig config;
    Propellers vtol_propellers{4};
    Propellers thrust_propeller{1};
    Ailerons elevons{2};

// Glasgow LatLon Height
#define INITIAL_LAT 55.8609825
#define INITIAL_LON -4.2488787
#define INITIAL_ALT 2600 // mm

    DroneSensors sensors{(DynamicObject&)*this,
        LatLonAlt{ INITIAL_LAT, INITIAL_LON, INITIAL_ALT }
    };

    void _setup_drone();
    void fake_ground_transform(boost::chrono::microseconds us);

public:
    StandaloneDrone(const char* config_file, Clock& clock) :
        DynamicObject(config_from_file_path(config_file), clock),
        config(config_from_file_path(config_file))
        { this->_setup_drone(); };
    
    void update(boost::chrono::microseconds us) override;

};

#endif // __STANDALONEDRONE_H__