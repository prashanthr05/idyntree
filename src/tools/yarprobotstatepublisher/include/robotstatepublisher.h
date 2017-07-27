/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file robotstatepublisher.h
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#ifndef YARP_ROBOT_STATE_PUBLISHER_H
#define YARP_ROBOT_STATE_PUBLISHER_H

#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>


/****************************************************************/
class YARPRobotStatePublisherModule : public yarp::os::RFModule
{
    yarp::dev::PolyDriver       m_ddtransformclient;
    yarp::dev::IFrameTransform       *m_iframetrans;
    double m_period;

    // Clock-related workaround
    bool m_usingNetworkClock;
    yarp::os::NetworkClock m_netClock;

public:
    YARPRobotStatePublisherModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
};

#endif

