#include "xil_printf.h"
#include <cstdarg>
#include <cstdio>

#include "Register.hpp"
#include "Logger.hpp"

//Logger::Logger( Register& reg )
//    : reg_ ( reg )
//    , control_word_(0x01)
//{
//    mutex_ = xSemaphoreCreateMutex();
//    
//    if ( mutex_ == NULL )
//    {
//        xil_printf( "Failed to create Logger control mutex.\n" );
//    }
//}

void Logger::set_register( Register* reg )
{
    reg_ = reg;
}


void Logger::set_log_control(uint8_t control)
{
    control_word_ = control | 0x01;
}


uint8_t Logger::read_log_control()
{
    return control_word_;
}


void Logger::log_error(const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_ERROR_TYPE, RED_TEXT, format, args);
    va_end(args);
}


void Logger::log_error(uint32_t error_code, const char *format, ...) const
{
    reg_->set_status(error_code);

    va_list args;
    va_start(args, format);
    log_va(LOG_ERROR_TYPE, RED_TEXT, format, args);
    va_end(args);
}


void Logger::log_warn(const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_WARN_TYPE, YELLOW_TEXT, format, args);
    va_end(args);
}


void Logger::log_debug(const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_DEBUG_TYPE, RESET_TEXT, format, args);
    va_end(args);
}


void Logger::log(LogType type, const char *format, ...) const
{
    if (type & control_word_)
    {
        va_list args;
        va_start(args, format);
        log_va(type, nullptr, format, args);
        va_end(args);
    }
}


void Logger::log(LogType type, char* color, const char *format, ...) const
{
    if (type & control_word_)
    {
        va_list args;
        va_start(args, format);
        log_va(type, color, format, args);
        va_end(args);
    }
}


void Logger::xvprintf(const char* format, va_list args) const
{
    char buf[256];
    vsnprintf(buf, sizeof(buf), format, args);
    xil_printf("%s", buf);
}


// Private helper method to centralize va_list handling and actual printing

void Logger::log_va(LogType type, const char* color, const char *format, va_list args) const
{
    if ( type & control_word_ )
    {
        // Print color code if provided
        if (color)
            xil_printf("%s", color);

        // Print prefix, e.g., "[ERROR] file.cpp:123: "
        //xil_printf(log_leaders_[log_type_to_index(type)], __FILE__, __LINE__);
        xil_printf(log_leaders_[type], __FILE__, __LINE__);

        // Print actual log message
        xvprintf(format, args);

        // Reset color if applied
        if (color)
            xil_printf("%s", RESET_TEXT);
    }
}

