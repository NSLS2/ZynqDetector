#include "GermaniumDetector.hpp"

int main()
{
    GermaniumDetector det;

    //det.base_->network_init();
    //det.base_->queue_init();
    det.base_->task_init();

    vTaskStartScheduler();

    while(1);
}
