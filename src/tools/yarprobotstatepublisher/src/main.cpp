/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#include <cstdlib>
#include <yarp/os/all.h>

#include "robotstatepublisher.h"

using namespace std;
using namespace yarp::os;


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork(10.0))
    {
        yError()<<"YARP network is not available";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    YARPRobotStatePublisherModule statepublisher;
    return statepublisher.runModule(rf);
}

