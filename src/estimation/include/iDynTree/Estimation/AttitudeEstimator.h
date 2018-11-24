/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Prashanth Ramadoss, Silvio Traversaro
 * email: silvio.traversaro@iit.it, prashanth.ramadoss@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef IDYNTREE_ATTITUDEESTIMATOR_H
#define IDYNTREE_ATTITUDEESTIMATOR_H
#include <Eigen/Core>
#include <iDynTree/Core/VectorFixSize.h>
namespace iDynTree
{   
    class Rotation;
    typedef iDynTree::Vector3 LinearAccelerometerMeasurements;
    typedef iDynTree::Vector3 GyroscopeMeasurements;
    typedef iDynTree::Vector3 MagnetometerMeasurements;
        
    typedef iDynTree::Vector4 Quaternion;
    typedef iDynTree::Vector3 RPY;
    
    // struct ekfParams;
    
    /**
     * Attitude Estimator
     * Implements Mahony filter, Quaternion Extended Kalman Filter,
     * Continuous-Discrete Lie Group Extended Kalaman Filters (on Rotation matrices)
     * 
     * Input - accelerometer, gyroscope measurements with optional magnetometer measurements
     * Output - orientation as a quaternion, rotation matrix or euler angles
     */
    class AttitudeEstimator
    {
    public:
        /**
         * Enumeration of implemented filters
         */
        enum filterType
        {
            /**
             * Mahony Filter
             */
            MAHONY, // 0
            
            /**
             * Quaternion based Extended Kalman Filter
             */
            QUATERNION_EKF, // 1
            
            /**
             * Continuous-Discrete-Lie Group- Extended Kalman Filter (Rotation matrix filter) 
             */
            CD_LG_EKF // 2
        };
        

        /**
         * @brief Constructor of Attitude Estimator
         * 
         * Implements an attitude estimation algorithm of the desired type
         * 
         * @param[in] useMagnetometerMeasurements flag to enable the use of magnetometer measurements
         * @param[in] type Type of attitude estimator 
         *                 Current available types are
         *                 MAHONY implementing a  Mahony filter,
         *                 QUATERNION_EKF implementing a quaternion based extended Kalman Filter
         *                 CD_LG_EKF implementing a continuous discrete extended Kalman filter on rotation matrices
         */
        AttitudeEstimator(bool useMagnetometerMeasurements, iDynTree::AttitudeEstimator::filterType type);
        
        /**
         * @brief Constructor of Attitude Estimator
         * 
         * Implements an attitude estimation algorithm of the desired type
         * 
         * @param[in] useMagnetometerMeasurements flag to enable the use of magnetometer measurements
         * @param[in] type Type of attitude estimator 
         *                 Current available types are
         *                 MAHONY implementing a  Mahony filter,
         *                 QUATERNION_EKF implementing a quaternion based extended Kalman Filter
         *                 CD_LG_EKF implementing a continuous discrete extended Kalman filter on rotation matrices
         * @param[in] ekf_params struct containing the necessary parameters to initialize the EKF
         */
        // AttitudeEstimator(bool useMagnetometerMeasurements, iDynTree::AttitudeEstimator::filterType type, iDynTree::ekfParams ekf_params);
        
        
        /**
         * @brief Update the filter with accelerometer and gyroscope measurements
         * 
         * @param[in] linAccMeas left trivialized 3D vector of linear proper sensor acceleration measuements
         * @param[in] gyroMeas left trivialized 3D vector of angular velocity measurements
         * 
         * @note left trivialized angular velocity means the angular velocity of body frame B with respect to an inertial fram A, expressed in frame B
         * @return true/false if successful/not
         */
        bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas);
        
        /**
         * @brief Update the filter with accelerometer, gyroscope and magnetometer measurements
         * 
         * @param[in] linAccMeas left trivialized 3D vector of linear proper sensor acceleration measuements
         * @param[in] gyroMeas left trivialized 3D vector of angular velocity measurements
         * @param[in] magMeas left trivialized 3D vector of magnetometer measurements
         * 
         * @note left trivialized angular velocity means the angular velocity of body frame B with respect to an inertial fram A, expressed in frame B
         * 
         * @return true/false if successful/not
         */
        bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas, const iDynTree::MagnetometerMeasurements& magMeas);
        
        
        /**
         * @brief Get orientation of the body with respect to inertial frame, in rotation matrix form
         * 
         * @param[out] rot Rotation matrix
         * @return true/false if successful/not
         */
        bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot);
        
        /**
         * @brief Get orientation of the body with respect to inertial frame, in quaternion form
         * 
         * @note quaternion has the form (real, imaginary) and is normalized
         * 
         * @param[out] q Quaternion
         * @return true/false if successful/not
         */
        bool getOrientationEstimateAsQuaternion(iDynTree::Quaternion& q);
        
        /**
         * @brief Get orientation of the body with respect to inertial frame, in Euler's RPY form
         * 
         * @param[out] rpy 3D vector containing roll pitch yaw angles
         * @return true/false if successful/not
         */
        bool getOrientationEstimateAsRPY(iDynTree::RPY& rpy);
    private:        
        LinearAccelerometerMeasurements m_accelerometerMeasurements;
        GyroscopeMeasurements m_gyroscopeMeasurements;
        MagnetometerMeasurements m_magnetometerMeasurements;
        
        bool m_useMagnetometerMeasurements;  ///< Flag to enable the use of magnetometer measurements for attitude estimation
        unsigned int m_estimatorType;        ///< Type of attitude estimator to run
        
        iDynTree::Quaternion m_estimatedOrientationAsQuaternion;
        iDynTree::Rotation m_estimatedOrientationAsRotationMatrix;
        iDynTree::RPY m_estimatedOrientationAsRPY;
        
        // std::unique_ptr<MahonyFilter> mahony;
        // std::unique_ptr<QuaternionEKF> qekf;
        // std::unique_ptr<CDLGEKF> cdlgekf;
        
    };
}
#endif  
