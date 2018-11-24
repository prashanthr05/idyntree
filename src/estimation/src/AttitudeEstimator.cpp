#include "iDynTree/Estimation/AttitudeEstimator.h"
namespace iDynTree 
{

AttitudeEstimator::AttitudeEstimator(bool useMagnetometerMeasurements, AttitudeEstimator::filterType type) : m_useMagnetometerMeasurements(useMagnetometerMeasurements),
                                                                                                             m_estimatorType(type)   
{
    if (m_estimatorType == filterType::QUATERNION_EKF || m_estimatorType == filterType::CD_LG_EKF)
    {
        throw std::runtime_error{"Please not you're choosing a EKF as filter type and not passing the \"ekfParams\" struct"};
    }
}

bool AttitudeEstimator::updateFilterWithMeasurements(const LinearAccelerometerMeasurements& linAccMeas, const GyroscopeMeasurements& gyroMeas, const MagnetometerMeasurements& magMeas)
{
    m_accelerometerMeasurements = linAccMeas;
    m_gyroscopeMeasurements = gyroMeas;
    m_magnetometerMeasurements = magMeas;
    return true;
}

bool AttitudeEstimator::updateFilterWithMeasurements(const LinearAccelerometerMeasurements& linAccMeas, const GyroscopeMeasurements& gyroMeas)
{
    if (m_useMagnetometerMeasurements)
    {
        throw std::runtime_error("The \"useMagnetometerMeasurements\" is set to true. The estimator also expects magnetometer measurements ");
    }
    m_accelerometerMeasurements = linAccMeas;
    m_gyroscopeMeasurements = gyroMeas;
    return true;
}

bool AttitudeEstimator::getOrientationEstimateAsQuaternion(Quaternion& q)
{
    q = m_estimatedOrientationAsQuaternion;
    return true;
}

bool AttitudeEstimator::getOrientationEstimateAsRotationMatrix(Rotation& rot)
{
    rot = m_estimatedOrientationAsRotationMatrix;
    return true;
}

bool AttitudeEstimator::getOrientationEstimateAsRPY(RPY& rpy)
{
    rpy = m_estimatedOrientationAsRPY;
    return true;
}

    
}
