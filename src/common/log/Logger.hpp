#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <memory>

#include <Register.hpp>

#include <xil_printf.h>

#define RED_TEXT "\x1b[31m"
#define YELLOW_TEXT "\x1b[33m"
#define REGULAR_TEXT "\x1b[0m"

class Logger {
public:
    enum LogType {
        LOG_ERROR_TYPE = 0x01,
        LOG_WARN_TYPE  = 0x02,
        LOG_DEBUG_TYPE = 0x04
    };

    Logger( std::shared_ptr<Register> reg);

    void set_log_control( uint8_t control );
    uint8_t read_log_control();

    void log_error(const char *format, ...);
    void log_warn(const char *format, ...);
    void log_debug(const char *format, ...);

private:

    std::shared_ptr<Register> reg_;

    uint8_t control_word;
    SemaphoreHandle_t control_mutex

    const char *log_leaders[3] =
    {
        "[ERROR] %s:%d: ",
        "[WARNING] %s:%d ",
        "[DEBUG] %s:%d : "
    };

    void log(LogType type, const char *format, ...);
    void log(LogType type, char* color, const char *format, ...);
};
