/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef ROTELLA_BASE_ESTIMATOR_H
#define ROTELLA_BASE_ESTIMATOR_H

#include <iDynTree/Estimation/FloatingBaseEstimator.h>
#include <iDynTree/Estimation/ExtendedKalmanFilter.h>
#include <iDynTree/Estimation/SchmittTrigger.h>

namespace iDynTree
{
    class RotellaBaseEstimator : public IFloatingBaseEstimator,
                                                    public DiscreteExtendedKalmanFilterHelper
    {
    public:
        virtual bool updateMeasurements() override;
        virtual bool predictSystemEvolution() override;

        virtual bool setIMUMeasurement(const std::string& imu_name,
                                                               const iDynTree::LinearAccelerometerMeasurements& acc,
                                                               const iDynTree::GyroscopeMeasurements& omega,
                                                               const iDynTree::MagnetometerMeasurements& mag) override;
        virtual bool setAccelerometerMeasurement(const std::string& accelerometer_name,
                                                                                const iDynTree::LinearAccelerometerMeasurements& acc) override;
        virtual bool setGyroscopeMeasurement(const std::string& gyroscope_name,
                                                                           const iDynTree::GyroscopeMeasurements& omega) override;
        virtual bool setContactWrenchMeasurement(const iDynTree::LinkIndex& link_index, const iDynTree::Wrench& contact_wrench) override;
        virtual bool setForwardKinematicsMeasurement(const std::vector<std::string>& joint_names, const iDynTree::JointDOFsDoubleArray& joint_positions) override;
        virtual bool setModel(const iDynTree::Model& robot_model) override;

        virtual bool getFloatingBasePose(iDynTree::Transform& w_H_b) override;
        virtual bool getFeetPose(std::unordered_map<std::string, iDynTree::Transform>& w_p_b) override;
        virtual bool getFloatingBaseState(const iDynTree::Span<double>& floating_base_state) override;
        virtual bool getFloatingBaseFullState(const iDynTree::Span<double>& floating_base_full_state) override;
        virtual bool getContactState(const iDynTree::LinkIndex& linkIndex, bool& contact_state) override;

        virtual bool getInternalStateSize() const override;
        virtual bool getInternalState(const iDynTree::Span<double>& filter_state) override;
        virtual bool setInternalState(const iDynTree::Span<double>& filter_state) override;
        virtual bool setInternalStateFloatingBasePose(const iDynTree::Transform& w_H_b) override;
        virtual bool setInternalStateFeetPose(const iDynTree::FrameIndex& foot_frame, const iDynTree::Transform& w_p_b) override;

        virtual bool ekf_f(const iDynTree::VectorDynSize& x_k,
                                      const iDynTree::VectorDynSize& u_k,
                                      iDynTree::VectorDynSize& xhat_k_plus_one) override;
        virtual bool ekf_h(const iDynTree::VectorDynSize& xhat_k_plus_one,
                             iDynTree::VectorDynSize& zhat_k_plus_one) override;
        virtual bool ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F) override;
        virtual bool ekfComputeJacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H) override;

        void setFeetInformation(double& nr_of_feet, std::vector<iDynTree::FrameIndex> feet_frames);

    private:

        struct
        {
            iDynTree::Position base_position;
            iDynTree::UnitQuaternion base_orientation; // quaternion
            iDynTree::LinearMotionVector3 base_linear_velocity; // left trivialized base linear velocity {^B}v_A,B
            iDynTree::AngularMotionVector3 base_angular_velocity; // left trivialize base angular velocity {^B}\omega_A,B
            std::unordered_map<iDynTree::FrameIndex, bool> feet_contact_state;
            std::unordered_map<iDynTree::FrameIndex, iDynTree::Position> feet_positions;
            std::unordered_map<iDynTree::FrameIndex, iDynTree::UnitQuaternion> feet_rotations;
            iDynTree::Vector3 accelerometer_bias;
            iDynTree::Vector3 gyro_bias;
            size_t nr_of_feet{2};
        }m_state;

        iDynTree::VectorDynSize m_x;
        iDynTree::VectorDynSize m_u;
        iDynTree::VectorDynSize m_y;
        size_t m_state_size;
        size_t m_input_size;
        size_t m_output_size;

        std::unordered_map<iDynTree::FrameIndex, iDynTree::SchmittTrigger> m_feet_contact_schmiit_triggers;
        std::unordered_map<iDynTree::FrameIndex, iDynTree::Wrench> m_feet_contact_wrenches;


    };
}

#endif
