/**
 * @file Logger.hpp
 * @brief Class definition of `Logger` — Linux version.
 * @details
 * printf + std::mutex. Replaces xil_printf + FreeRTOS semaphore.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <mutex>

//===========================================================================//

#define RED_TEXT    "\x1b[31m"
#define YELLOW_TEXT "\x1b[33m"
#define RESET_TEXT  "\x1b[0m"

//===========================================================================//

class Register;

//===========================================================================//

class Logger {
public:
    enum LogType {
        LOG_ERROR_TYPE = 0x1,
        LOG_WARN_TYPE  = 0x2,
        LOG_DEBUG_TYPE = 0x4
    };

    /**
     * @brief Receive reference to register.
     */
    void set_register( Register* reg );
    
    /**
     * @brief Set log control value.
     */
    void set_log_control( uint8_t control );

    /**
     * @brief Read log control value.
     */
    uint8_t read_log_control() const;

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
    void log_va( LogType type, const char* color, const char *format, va_list args ) const;

    Register*           reg_ = nullptr;
    uint8_t             control_word_ = 0x07;  ///< all levels enabled by default
    mutable std::mutex  mutex_;
};

//===========================================================================//
