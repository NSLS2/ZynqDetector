#include "GeRM.hpp"

int main()
{
    GeRM det;

    det.network_init();
    det.queue_init();
    det.task_init();

    vTaskStartScheduler();

    while(1);
}
