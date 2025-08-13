/**
 * @file detector_main.cpp
 * @brief Starting point of the program.
 * @details
 * This file defines an object of `GermaniumDetector` class,
 * does initialization, and wait forever.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#include "GermaniumDetector.hpp"

int main()
{
    GermaniumDetector det;

    det.base_->create_queues();

    det.base_->network_init();

    det.base_->create_tasks();

    vTaskStartScheduler();

    for(;;);
}
