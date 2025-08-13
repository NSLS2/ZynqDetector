/**
 * @file Logger.cpp
 * @brief Member function definitions of `Logger`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "xil_printf.h"
#include <cstdarg>
#include <cstdio>

#include "Register.hpp"
#include "Logger.hpp"

//===========================================================================//

/**
 * @brief Receive reference to the register. To be called by GermaniumDetector.
 * @param reg Reference to the register.
 */
void Logger::set_register( Register* reg )
{
    reg_ = reg;
}

/** 
 * @brief Set log control value.
 * @param control Log control value.
 */
void Logger::set_log_control(uint8_t control)
{
    control_word_ = control | 0x01;
}

/** 
 * @brief Read log control value.
 */
uint8_t Logger::read_log_control()
{
    return control_word_;
}

/** 
 * @brief Log error.
 * @param format Log format.
 */
void Logger::log_error(const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_ERROR_TYPE, RED_TEXT, format, args);
    va_end(args);
}

/** 
 * @brief Log error with error code.
 * @param error_code Error code.
 * @param format Log format.
 */
void Logger::log_error(uint32_t error_code, const char *format, ...) const
{
    reg_->set_status(error_code);

    va_list args;
    va_start(args, format);
    log_va(LOG_ERROR_TYPE, RED_TEXT, format, args);
    va_end(args);
}

/**
 * @brief Log warning.
 * @param Log format.
 */
void Logger::log_warn(const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_WARN_TYPE, YELLOW_TEXT, format, args);
    va_end(args);
}

/**
 * @brief Log debug information.
 * @param format Log format.
 */
void Logger::log_debug(const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    log_va(LOG_DEBUG_TYPE, RESET_TEXT, format, args);
    va_end(args);
}

/**
 * @brief Log general information.
 * @param type Log type.
 * @param format Log format.
 */
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

/**
 * @brief Log general information with color.
 * @param type Log type.
 * @param color Log print color.
 * @param format Log format.
 */
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

/**
 * @brief Print.
 */
void Logger::xvprintf(const char* format, va_list args) const
{
    char buf[256];
    vsnprintf(buf, sizeof(buf), format, args);
    xil_printf("%s", buf);
}

/**
 * @brief Private helper method to centralize va_list handling and actual
 *        printing.
 * @param type Log type.
 * @param color Log print color.
 * @param format Log format.
 * @param args Information to be printed.
 */
void Logger::log_va(LogType type, const char* color, const char *format, va_list args) const
{
    if ( type & control_word_ )
    {
        ///< Print color code if provided
        if (color)
            xil_printf("%s", color);

        xil_printf(log_leaders_[type], __FILE__, __LINE__);

        ///< Print actual log message
        xvprintf(format, args);

        ///< Reset color if applied
        if (color)
            xil_printf("%s", RESET_TEXT);
    }
}

//===========================================================================//
