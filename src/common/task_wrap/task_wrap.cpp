
#include <functional>
#include "FreeRTOS.h"
#include <memory>
#include "task.h"

#include "task_wrap.hpp"

void task_wrapper(void* pvParameters)
{
    ///auto task_func = std::unique_ptr<std::function<void()>>(static_cast<std::function<void()>*>(pvParameters));
    ///if (task_func)
    ///{
    ///    (*task_func)();  // Call the actual function
    ///}
    ///vTaskDelete(nullptr);
    auto* func_ptr = static_cast<std::function<void()>*>(pvParameters);
    (*func_ptr)();  // call the function
    delete func_ptr; // clean up if dynamically allocated
    vTaskDelete(NULL); // clean up task

}
