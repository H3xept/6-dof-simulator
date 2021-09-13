#ifndef __CLOCK_H__
#define __CLOCK_H__

#include <boost/chrono.hpp>

class Clock {
public: 
    virtual boost::chrono::microseconds get_current_time_us() = 0;
};

#endif // __CLOCK_H__