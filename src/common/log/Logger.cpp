#include "Logger.hpp"

Logger::Logger()
    : reg_(reg)
    , control_word(0x01)
{
    control_mutex = xSemaphoreCreateMutex();
    
    if (control_mutex == NULL) {
        log_error("Failed to create Logger control mutex.\n");
    }
}

void Logger::set_log_control(uint8_t control)
{
    control_word = control | 0x01;
}

uint8_t Logger::read_log_control()
{
    return control_word;
}

void Logger::log_error(const char *format, ..., uint32_t error_code)
{
    reg_->set_status( error_code );
    log(LOG_ERROR_TYPE, RED_TEXT, format, __VA_ARGS__);
}

void Logger::log_warn(const char *format, ...)
{
    log(LOG_WARN_TYPE, YELLOW_TEXT, format, __VA_ARGS__);
}

void Logger::log_debug(const char *format, ...)
{
    log(LOG_DEBUG_TYPE, REGULAR_TEXT, format, __VA_ARGS__);
}


void Logger::log(LogType type, const char *format, ...)
{
    if ( type & control_word )
    {
        //xvprintf(log_leaders[type], __PERFECT_FUNCTION__);
        xvprintf( log_leaders[type], __FILE__, __LINE__ );
        va_list args;
        va_start(args, format);
        xvprintf(format, args);
        va_end(args);
    }
}

void Logger::log(LogType type, char* color, const char *format, ...)
{
    if ( type & control_word )
    {
        xvprintf( color );
        //xvprintf(log_leaders[type], __PERFECT_FUNCTION__);
        xvprintf( log_leaders[type], __FILE__, __LINE__ );
        va_list args;
        va_start(args, format);
        xvprintf(format, args);
        va_end(args);
        xvprintf( RESET_TEXT );
    }
}


