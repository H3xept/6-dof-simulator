#ifndef __TIMEHANDLER_H__
#define __TIMEHANDLER_H__

class TimeHandler {
public:
    virtual void update(double dt) = 0;
};

#endif // __TIMEHANDLER_H__