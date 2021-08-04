#ifndef __CONSOLELOGGER_H__
#define __CONSOLELOGGER_H__

#include <string>
#include "../Interfaces/Logger.h"

enum LoggerMode { NORMAL, DEBUG };

class ConsoleLogger : public Logger {
public:
    ~ConsoleLogger();
    void log(std::string s) override;
    void set_debug(bool debug) override;
    bool debug_mode_active() override;
private:
    LoggerMode log_mode;
};

#endif // __CONSOLELOGGER_H__