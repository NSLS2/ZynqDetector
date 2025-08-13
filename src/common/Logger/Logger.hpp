/**
 * @file Logger.hpp
 * @brief Class definition of `Logger`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <memory>
#include <xil_printf.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "Register.hpp"

//===========================================================================//

#define RED_TEXT    "\x1b[31m"
#define YELLOW_TEXT "\x1b[33m"
#define RESET_TEXT  "\x1b[0m"

//===========================================================================//

class Logger {
public:
    enum LogType {
        LOG_ERROR_TYPE = 0x0,
        LOG_WARN_TYPE  = 0x1,
        LOG_DEBUG_TYPE = 0x2
    };

    /**
     * @brief Receive reference to register. To be called by GermaniumDetector.
     */
    void set_register( Register* reg );
    
    /**
     * @brief Set log control value.
     */
    void set_log_control( uint8_t control );

    /**
     * @brief Read log control value.
     */
    uint8_t read_log_control();

    /**
     * @brief Log error.
     */
    void log_error( const char *format, ... ) const;


    /**
     * @brief Log error with error code.
     */
    void log_error( uint32_t error_code, const char *format, ... ) const;

    /**
     * @brief Log warning.
     */
    void log_warn( const char *format, ... ) const;

    /**
     * @brief Log debug information.
     */
    void log_debug( const char *format, ... ) const;

private:

    /**
     * @brief Print.
     */
    void xvprintf(const char* format, va_list args) const;

    /**
     * @brief General log function.
     */
    void log_va( LogType type, const char* color, const char *format, va_list args ) const;

    Register* reg_;

    uint8_t control_word_;
    xSemaphoreHandle mutex_;

    const char *log_leaders_[3] =
    {
        "[ERROR] %s:%d: ",
        "[WARNING] %s:%d ",
        "[DEBUG] %s:%d : "
    };

    /**
     * @brief Log general information.
     */
    void log(LogType type, const char *format, ...) const;

    /**
     * @brief Log general information with color.
     */
    void log(LogType type, char* color, const char *format, ...) const;

};


