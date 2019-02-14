/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_FLOATINGBASE_ESTIMATOR_H
#define IDYNTREE_FLOATINGBASE_ESTIMATOR_H

#include <unordered_map>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/JointState.h>

namespace iDynTree
{
    typedef iDynTree::Vector3 LinearAccelerometerMeasurements;
    typedef iDynTree::Vector3 GyroscopeMeasurements;
    typedef iDynTree::Vector3 MagnetometerMeasurements;

    typedef iDynTree::Vector4 UnitQuaternion;
    typedef iDynTree::Vector3 RPY;
    typedef iDynTree::Vector3 FootPosition;

    class IFloatingBaseEstimator
    {
    public:
        virtual ~IFloatingBaseEstimator() = 0;
        virtual bool updateMeasurements() = 0;
        virtual bool predictSystemEvolution() = 0;

        virtual bool setIMUMeasurement(const std::string& imu_name,
                                                               const iDynTree::LinearAccelerometerMeasurements& acc,
                                                               const iDynTree::GyroscopeMeasurements& omega,
                                                               const iDynTree::MagnetometerMeasurements& mag) = 0;
        virtual bool setAccelerometerMeasurement(const std::string& accelerometer_name,
                                                                                const iDynTree::LinearAccelerometerMeasurements& acc) = 0;
        virtual bool setGyroscopeMeasurement(const std::string& gyroscope_name,
                                                                           const iDynTree::GyroscopeMeasurements& omega) = 0;
        virtual bool setContactWrenchMeasurement(const iDynTree::LinkIndex& link_index, const iDynTree::Wrench& contact_wrench) = 0;
        virtual bool setForwardKinematicsMeasurement(const std::vector<std::string>& joint_names, const iDynTree::JointDOFsDoubleArray& joint_positions) = 0;
        virtual bool setModel(const iDynTree::Model& robot_model) = 0;

        virtual bool getFloatingBasePose(iDynTree::Transform& w_H_b) = 0;
        virtual bool getFeetPose(std::unordered_map<std::string, iDynTree::Transform>& w_p_b) = 0;
        virtual bool getFloatingBaseState(const iDynTree::Span<double>& floating_base_state) = 0;
        virtual bool getFloatingBaseFullState(const iDynTree::Span<double>& floating_base_full_state) = 0;
        virtual bool getContactState(const iDynTree::LinkIndex& linkIndex, bool& contact_state) = 0;

        virtual bool getInternalStateSize() const = 0;
        virtual bool getInternalState(const iDynTree::Span<double>& filter_state) = 0;
        virtual bool setInternalState(const iDynTree::Span<double>& filter_state) = 0;
        virtual bool setInternalStateFloatingBasePose(const iDynTree::Transform& w_H_b) = 0;
        virtual bool setInternalStateFeetPose(const iDynTree::FrameIndex& foot_frame, const iDynTree::Transform& w_p_b) = 0;
    };
}

#endif
