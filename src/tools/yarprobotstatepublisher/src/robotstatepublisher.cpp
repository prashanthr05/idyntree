/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file coordinator.cpp
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include <yarp/math/Math.h>

#include "robotstatepublisher.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/************************************************************/
YARPRobotStatePublisherModule::YARPRobotStatePublisherModule(): m_iframetrans(nullptr),
                                                                m_usingNetworkClock(false)
{
}


/************************************************************/
bool YARPRobotStatePublisherModule::configure(ResourceFinder &rf)
{
    string name="yarprobotstatepublisher";
    string robot=rf.check("robot",Value("isaacSim")).asString();
    m_period=rf.check("period",Value(0.016)).asDouble();

    Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    pTransformclient_cfg.put("local", "/transformClientTest");
    pTransformclient_cfg.put("remote", "/transformServer");

    bool ok_client = m_ddtransformclient.open(pTransformclient_cfg);
    if (!ok_client)
    {
        yError()<<"Problem in opening the transformClient device";
        close();
        return false;
    }

    if (!m_ddtransformclient.view(m_iframetrans))
    {
        yError()<<"IFrameTransform I/F is not implemented";
        close();
        return false;
    }

    // If YARP is using a network clock, writing on a ROS topic is not working
    // Workaround: explicitly instantiate a network clock to read the time from gazebo
    if( yarp::os::NetworkBase::exists("/clock") )
    {
        m_usingNetworkClock = true;
        m_netClock.open("/clock");
    }

    return true;
}


/************************************************************/
bool YARPRobotStatePublisherModule::close()
{

    if (m_ddtransformclient.isValid())
    {
        yInfo()<<"Closing the tf device";
        m_ddtransformclient.close();
        m_iframetrans = nullptr;
    }

    m_tfMarkersTopic.close();

    if (m_pRosNode)
    {
        delete m_pRosNode;
        m_pRosNode = nullptr;
    }

    return true;
}


/************************************************************/
double YARPRobotStatePublisherModule::getPeriod()
{
    return m_period;
}



/************************************************************/
bool BalanceCoordinator::updateModule()
{
    // Publish a green marker on end effectors origin
    publishEEMarkersToTF();

    return true;
}

/************************************************************/
void BalanceCoordinator::publishEEMarkersToTF()
{
    double yarpTimeStamp;

    if (m_usingNetworkClock)
    {
        yarpTimeStamp = m_netClock.now();
    }
    else
    {
       yarpTimeStamp = yarp::os::Time::now();
    }

    uint64_t time;
    uint64_t nsec_part;
    uint64_t sec_part;
    TickTime ret;
    time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    nsec_part = (time % 1000000000UL);
    sec_part = (time / 1000000000UL);

    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    visualization_msgs_MarkerArray& markerarray = m_tfMarkersTopic.prepare();

    markerarray.markers.clear();

    visualization_msgs_Marker markerLEE;
    markerLEE.header.stamp.sec = (yarp::os::NetUint32) sec_part;
    markerLEE.header.stamp.nsec = (yarp::os::NetUint32) nsec_part;
    markerLEE.ns = "balancing_namespace";
    markerLEE.type = visualization_msgs_Marker::SPHERE;
    markerLEE.action = visualization_msgs_Marker::ADD;
    markerLEE.id = 1;
    markerLEE.pose.position.x = 0.0;
    markerLEE.pose.position.y = 0.0;
    markerLEE.pose.position.z = 0.0;
    markerLEE.pose.orientation.x = 0.0;
    markerLEE.pose.orientation.y = 0.0;
    markerLEE.pose.orientation.z = 0.0;
    markerLEE.pose.orientation.w = 1.0;
    markerLEE.scale.x = 0.01;
    markerLEE.scale.y = 0.01;
    markerLEE.scale.z = 0.01;
    markerLEE.color.a = 1.0;
    markerLEE.color.r = 0.0;
    markerLEE.color.g = 1.0;
    markerLEE.color.b = 0.0;

    // l_gripper marker
    markerLEE.header.frame_id = "l_gripper";
    markerarray.markers.push_back(markerLEE);

    // r_gripper marker
    visualization_msgs_Marker markerREE = markerLEE;
    markerREE.id = 2;
    markerREE.header.frame_id = "r_gripper";
    markerarray.markers.push_back(markerREE);


    // Desired l_gripper location
    visualization_msgs_Marker markerLEEdes = markerLEE;
    markerLEEdes.id = 3;
    markerLEEdes.color.a = 1.0;
    markerLEEdes.color.r = 1.0;
    markerLEEdes.color.g = 0.0;
    markerLEEdes.color.b = 0.0;
    markerLEEdes.header.frame_id = "l_gripper";
    markerarray.markers.push_back(markerLEEdes);

    // Desired r_gripper location
    visualization_msgs_Marker markerREEdes = markerREE;
    markerREEdes.id = 4;
    markerREEdes.color.a = 1.0;
    markerREEdes.color.r = 1.0;
    markerREEdes.color.g = 0.0;
    markerREEdes.color.b = 0.0;
    markerREEdes.header.frame_id = "r_gripper";
    markerarray.markers.push_back(markerREEdes);

    // Publish
    m_tfMarkersTopic.write();
}
