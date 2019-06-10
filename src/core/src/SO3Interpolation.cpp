/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/SO3Interpolation.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iostream>
#include <string>
#include <cmath>

using iDynTree::toEigen;

iDynTree::SO3Interpolation::SO3Interpolation()
{
     m_Id3 = iDynTree::Rotation::Identity();
}

bool iDynTree::SO3Interpolation::setInitialConditions(const iDynTree::AngVelocity& omega0,
                                                           const iDynTree::AngAcceleration& alpha0)
{
    m_omega0 = omega0;
    m_alpha0 = alpha0;
    return true;
}


bool iDynTree::SO3Interpolation::setData(const iDynTree::VectorDynSize& knot_times,
                                         const std::vector<iDynTree::Rotation>& knot_points)
{
    if ((knot_times.size() == 0) && (knot_points.size() == 0))
    {
        iDynTree::reportError("SO3Interpolation", "setData", "The input data are empty.");
        return false;
    }

    if (knot_times.size() != knot_points.size())
    {
        iDynTree::reportError("SO3Interpolation", "setData", "The input data are expected to have the same size.");
        return false;
    }

    if (knot_times.size() < 2)
    {
        iDynTree::reportError("SO3Interpolation", "setData", "Atleast two data points required to compute the interpolant.");
        return false;
    }

    m_time.resize(knot_times.size());
    m_R.resize(knot_points.size());

    m_coefficient_a.resize(knot_times.size());
    m_coefficient_b.resize(knot_times.size());
    m_coefficient_c.resize(knot_times.size());

    m_r.resize(knot_times.size());
    m_rcross.resize(knot_times.size());
    m_A.resize(knot_times.size());

    m_R = knot_points;
    m_time = knot_times;

    if (!this->computeTangentSpaceVectors())
    {
        return false;
    }

    return this->computeCoefficients();
}


bool iDynTree::SO3Interpolation::computeTangentSpaceVectors()
{
    bool trace_condition_satisfied{true};
    for (size_t rot = 1; rot < m_R.size(); rot++)
    {
        iDynTree::Rotation deltaR = iDynTree::Rotation::compose(m_R[rot - 1].inverse(), m_R[rot]);

        if (toEigen(deltaR).trace() == -1)
        {
            trace_condition_satisfied = false;
            break;
        }

        m_rcross[rot] = this->SO3Log(deltaR);
        m_r[rot] = this->so3vee(m_rcross[rot]);
        double theta = toEigen(m_r[rot]).norm();
        double a = (1 - std::cos(theta))/(theta*theta);
        double b = (theta - std::sin(theta))/std::pow(theta, 3);

        toEigen(m_A[rot]) = toEigen(m_Id3) - a*toEigen(m_rcross[rot]) + b*toEigen(m_rcross[rot])*toEigen(m_rcross[rot]);
    }

    if (!trace_condition_satisfied)
    {
        reportError("SO3Interpolation", "computeTangentSpaceVectors", "Trace(R) = -1 condition not satisfied. Unable to interpolate for the given input sequence.");
        return false;
    }
    return true;
}

bool iDynTree::SO3Interpolation::computeCoefficients()
{
    // initialize
    m_coefficient_c[1] = m_omega0;
    toEigen(m_coefficient_b[1]) = toEigen(m_alpha0)/2;
    toEigen(m_coefficient_a[1]) = toEigen(m_r[1]) - toEigen(m_coefficient_b[1]) - toEigen(m_coefficient_c[1]);

    for (size_t iter = 2; iter < m_R.size(); iter++)
    {
        // temporary variables
        iDynTree::Vector3 s, t, u;
        s = m_r[iter];
        toEigen(t) = 3*toEigen(m_coefficient_a[iter - 1]) + 2*toEigen(m_coefficient_b[iter - 1]) + toEigen(m_coefficient_c[iter - 1]);
        toEigen(u) = 6*toEigen(m_coefficient_a[iter - 1]) + 2*toEigen(m_coefficient_b[iter - 1]);

        double stdot = toEigen(s).dot(toEigen(t));
        double smod = toEigen(s).norm();
        iDynTree::Vector3 s_cross_t, s_cross_u, s_cross_s_cross_u, s_cross_s_cross_t, t_cross_s_cross_t;
        toEigen(s_cross_t) = toEigen(s).cross(toEigen(t));
        toEigen(s_cross_u) = toEigen(s).cross(toEigen(u));
        toEigen(s_cross_s_cross_u) = toEigen(s).cross(toEigen(s_cross_u));
        toEigen(s_cross_s_cross_t) = toEigen(s).cross(toEigen(s_cross_t));
        toEigen(t_cross_s_cross_t) = toEigen(t).cross(toEigen(s_cross_t));

        double p_ = stdot/std::pow(smod, 4)*(2*std::cos(smod) + smod*std::sin(smod) - 2);
        double q_ = (1 - std::cos(smod))/(smod*smod);
        double r_ = stdot/std::pow(smod, 5)*(3*std::sin(smod) - smod*std::cos(smod) - 2*smod);
        double s_ = (smod - std::sin(smod))/std::pow(smod, 3);

        // compute coefficients
        toEigen(m_coefficient_c[iter]) = toEigen(m_A[iter - 1])*toEigen(m_coefficient_c[iter - 1]);
        toEigen(m_coefficient_b[iter]) = (toEigen(u) - p_*toEigen(s_cross_t) - q_*toEigen(s_cross_u) +
                                          r_*toEigen(s_cross_s_cross_t) + s_*(toEigen(t_cross_s_cross_t) + toEigen(s_cross_s_cross_u)))/2;
        toEigen(m_coefficient_a[iter]) = toEigen(s) - toEigen(m_coefficient_b[iter]) - toEigen(m_coefficient_c[iter]);
    }

    return true;
}

iDynTree::Rotation iDynTree::SO3Interpolation::evaluatePoint(double t)
{
    iDynTree::Rotation R_t;
    if (m_time.size() == 0)
    {
        iDynTree::reportError("SO3Interpolation", "evaluatePoint",
                              "You will have to setData() first, the returned data should not be considered.");
        return R_t;
    }

    if (t < m_time(0))
    {
        return m_R[0];
    }

    if (t >= m_time(m_time.size() - 1))
    {
        return m_R[m_R.size() - 1];
    }

    // find index of t such that,  t_{i - 1} \leq t \leq t_{i}
    size_t coeffIndex = 0;
    while ( (coeffIndex < m_time.size()) && (t >= m_time(coeffIndex)))
    {
        coeffIndex++;
    }


    double tau = (t - m_time(coeffIndex - 1))/(m_time(coeffIndex) - m_time(coeffIndex-1));
    iDynTree::Vector3 velocity;
    toEigen(velocity) = std::pow(tau, 3)*toEigen(m_coefficient_a[coeffIndex]) +
                        (tau*tau)*toEigen(m_coefficient_b[coeffIndex]) +
                        tau*toEigen(m_coefficient_c[coeffIndex]);
    R_t = iDynTree::Rotation::compose(m_R[coeffIndex - 1], this->SO3Exp(this->so3hat(velocity)));
    return R_t;
}


iDynTree::Matrix3x3 iDynTree::SO3Interpolation::so3hat(const iDynTree::Vector3& r)
{
    iDynTree::Matrix3x3 so3;
    so3(0, 1) = -r(2);
    so3(0, 2) = r(1);
    so3(1, 0) = r(2);
    so3(1, 2) = -r(0);
    so3(2, 0) = -r(1);
    so3(2, 1) = r(0);

    return so3;
}

iDynTree::Vector3 iDynTree::SO3Interpolation::so3vee(const iDynTree::Matrix3x3& so3)
{
    iDynTree::Vector3 r;
    r(0) = so3(2, 1);
    r(1) = so3(0, 2);
    r(2) = so3(1, 0);

    return r;
}

iDynTree::Rotation iDynTree::SO3Interpolation::SO3Exp(const iDynTree::Matrix3x3& so3)
{
    iDynTree::Rotation R;
    iDynTree::Vector3 r = so3vee(so3);
    double theta = toEigen(r).norm();

    toEigen(R) = toEigen(m_Id3) + (std::sin(theta)/theta)*toEigen(so3) +
                 ((1 - std::cos(theta))/ (theta*theta))*toEigen(so3)*toEigen(so3);

    return R;
}

iDynTree::Matrix3x3 iDynTree::SO3Interpolation::SO3Log(const iDynTree::Rotation& R)
{
    // TODO: check Tr(R) = -1 conditions and check Tr(R) \neq -1 conditions
    iDynTree::Matrix3x3 A;
    toEigen(A) = (toEigen(R) - toEigen(R.inverse()))/2;
    double theta = std::sqrt(-((toEigen(A)*toEigen(A)).trace())/2);

    iDynTree::Matrix3x3 out;
    toEigen(out) = (std::asin(theta)/theta)*toEigen(A);

    return out;
}


