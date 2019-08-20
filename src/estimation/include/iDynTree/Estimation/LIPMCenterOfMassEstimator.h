/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef LIPM_COM_ESTIMATOR_H
#define LIPM_COM_ESTIMATOR_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <memory>

class LIPMCenterOfMassEstimator
{
public:
    LIPMCenterOfMassEstimator();

    // set parameters
    bool setCenterOfMassHeight(const double& com_z);
    bool setGravity(const double& g);

    // initialize estimator
    bool initialize();

    // set measurements
    bool setIMUMeasurements(const std::string& imu_name, const iDynTree::Vector3& acc, const iDynTree::Vector3& gyro);
    bool setKinematicMeasurements(const iDynTree::VectorDynSize& joint_positions);

    //optional
    //// bool setContactWrenchMeasurements(const iDynTree::FrameIndex& foot, const iDynTree::Wrench& contact_wrench);

    // run step
    bool doEstimate();

    // get states
    bool getCOMPosition(iDynTree::Vector3& com_position);
    bool getCOMOffset(iDynTree::Vector3& com_offset);
    bool getCOMVelocity(iDynTree::Vector3& com_velocity);

private:
        class LIPMCoMEstimatorImpl;
        std::unique_ptr<LIPMCoMEstimatorImpl> m_pimpl;
};

#endif
