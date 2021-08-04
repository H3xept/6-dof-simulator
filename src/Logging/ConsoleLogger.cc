#include <string>
#include "ConsoleLogger.h"

ConsoleLogger::~ConsoleLogger() {}

void ConsoleLogger::log(std::string s)  {
    fprintf(stdout, "%s\n", s.c_str());
}

void ConsoleLogger::set_debug(bool debug) {
    this->log_mode = debug ? LoggerMode::DEBUG : LoggerMode::NORMAL;
    this->debug_log("Debug logs active");
}

bool ConsoleLogger::debug_mode_active() {
    return this->log_mode == LoggerMode::DEBUG;
}