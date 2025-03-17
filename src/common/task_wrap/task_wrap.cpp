
#include <functional>
#include "FreeRTOS.h"
#include "task.h"

#include "task_wrap.hpp"

void task_wrapper(void* pvParameters)
{
    auto taskFunc = std::unique_ptr<std::function<void()>>(static_cast<std::function<void()>*>(pvParameters));
    if (task_func)
    {
        (*task_func)();  // Call the actual function
    }
    vTaskDelete(nullptr);
}
