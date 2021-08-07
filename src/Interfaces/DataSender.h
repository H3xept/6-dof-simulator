#ifndef __DATASENDER_H__
#define __DATASENDER_H__

#include <stdio.h>

class DataSender {
public:
    ~DataSender() {}
    virtual void send_data(const char* buff, size_t len) = 0;
};

#endif // __DATASENDER_H__