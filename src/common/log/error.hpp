#pragma once

#include <string>
#include <iostream>
#include <string>

#include "fpga.hpp"
#include "zynq_detector.hpp"

 

class NetException : public std::exception {
private:
    std::string message;
    int code;

public:
    NetException( const std::string& msg, int code )
        : message(msg), code(code) {}

    const char* what() const noexcept override
    {
        return message.c_str();
    }

    int get_error_code() const
    {
        return code;
    }
};

