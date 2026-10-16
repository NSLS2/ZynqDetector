/**
 * @file Logger.cpp
 * @brief Member function definitions of `Logger` — Linux version.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include <cstdio>
#include <cstdarg>

#include "Register.hpp"
#include "Logger.hpp"

//===========================================================================//

void Logger::set_register( Register* reg )
{
    reg_ = reg;
}

//===========================================================================//

void Logger::set_log_control( uint8_t control )
{
    control_word_ = control | 0x01;  ///< always enable errors
}

//===========================================================================//

uint8_t Logger::read_log_control() const
{
    return control_word_;
}

//===========================================================================//

void Logger::log_error( const char *format, ... ) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_ERROR_TYPE, RED_TEXT, format, args);
    va_end(args);
}

//===========================================================================//

void Logger::log_error( uint32_t error_code, const char *format, ... ) const
{
    if (reg_) {
        reg_->set_status(error_code);
    }

    va_list args;
    va_start(args, format);
    log_va(LOG_ERROR_TYPE, RED_TEXT, format, args);
    va_end(args);
}

//===========================================================================//

void Logger::log_warn( const char *format, ... ) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_WARN_TYPE, YELLOW_TEXT, format, args);
    va_end(args);
}

//===========================================================================//

void Logger::log_debug( const char *format, ... ) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_DEBUG_TYPE, RESET_TEXT, format, args);
    va_end(args);
}

//===========================================================================//

void Logger::log_va( LogType type, const char* color, const char *format, va_list args ) const
{
    if ( !(type & control_word_) )
        return;

    std::lock_guard<std::mutex> lock(mutex_);

    if (color)
        printf("%s", color);

    static const char* leaders[] = {
        nullptr,
        "[ERROR] ",
        "[WARN]  ",
        nullptr,
        "[DEBUG] "
    };

    int idx = 0;
    if (type == LOG_ERROR_TYPE) idx = 1;
    else if (type == LOG_WARN_TYPE) idx = 2;
    else if (type == LOG_DEBUG_TYPE) idx = 4;

    if (idx > 0 && leaders[idx])
        printf("%s", leaders[idx]);

    vprintf(format, args);
    printf("\n");

    if (color)
        printf("%s", RESET_TEXT);

    fflush(stdout);
}

//===========================================================================//
