#pragma once

#include "FreeRTOS.h"
#include "task.h"

void task_wrapper(void* pvParameters);

/*
// Use case
#include <functional>
#include <iostream>
#include "task_wrap.hpp"

class MyClass {
public:
    MyClass()
    {
        task_func = [this]() { task_function(); };
        xTaskCreate(task_wrapper, "MyTask", 1000, &task_func, 1, &taskHandle);
    }

private:
    TaskHandle_t taskHandle;
    std::function<void()> task_func;

    friend void task_wrapper(void* pvParameters);
};
*/
