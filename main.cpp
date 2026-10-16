/**
 * @file detector_main.cpp
 * @brief Starting point of the program — Linux version.
 * @details
 * Creates GermaniumDetector (constructor initializes everything),
 * then enters the blocking ZMQ command loop.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include <cstdio>
#include <csignal>
#include <iostream>

#include "GermaniumDetector.hpp"

//===========================================================================//

static GermaniumDetector* g_det = nullptr;

static void signal_handler( int /*sig*/ )
{
    if (g_det) {
        g_det->stop();
    }
}

//===========================================================================//

int main( int argc, char* argv[] )
{
    if ( argc != 2 )
    {
        std::cerr << "Usage: " << argv[0] << " <192|384>\n";
	return 1;
    }

    int nelm = std::atoi(argv[1]);
        if ( nelm != 192 && nelm != 384 )
        {
	        std::fprintf(stderr, "Error: NELM must be 192 or 384\n");
		    return 1;
        }

    std::cerr << "GermaniumDetector starting with NELM=" << nelm << "...\n";

    GermaniumDetector det(nelm);
    g_det = &det;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    det.run();

    printf("GermaniumDetector stopped.\n");
    return 0;
}

//===========================================================================//
