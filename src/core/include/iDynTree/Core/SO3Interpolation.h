
/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SO3INTERP_H
#define IDYNTREE_SO3INTERP_H

#include <vector>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>

namespace iDynTree
{
    class SO3Interpolation
    {
    public:
        SO3Interpolation();

        bool setData(const iDynTree::VectorDynSize& knot_times, const std::vector<iDynTree::Rotation>& knot_points);
        // omega0 - left trivialized angular velocity, angular velocity of body with respect to inertial frame, expressed in the body frame
        bool setInitialConditions(const iDynTree::AngVelocity& omega0, const iDynTree::AngAcceleration& alpha0);
        iDynTree::Rotation evaluatePoint(double t);

    private:
        bool computeTangentSpaceVectors();
        bool computeCoefficients();

        iDynTree::Rotation SO3Exp(const iDynTree::Matrix3x3& so3);
        iDynTree::Matrix3x3 SO3Log(const iDynTree::Rotation& R);
        iDynTree::Vector3 so3vee(const iDynTree::Matrix3x3& so3);
        iDynTree::Matrix3x3 so3hat(const iDynTree::Vector3& r);

        iDynTree::VectorDynSize m_time;
        std::vector<iDynTree::Rotation> m_R;

        std::vector<iDynTree::Matrix3x3> m_rcross;
        std::vector<iDynTree::Vector3> m_r;
        std::vector<iDynTree::Matrix3x3> m_A; ///< Angular velocity map

        iDynTree::Rotation m_Id3;

        // coefficients for the cubic spline
        std::vector<iDynTree::Vector3> m_coefficient_a;
        std::vector<iDynTree::Vector3> m_coefficient_b;
        std::vector<iDynTree::Vector3> m_coefficient_c;

        iDynTree::AngVelocity m_omega0;
        iDynTree::AngAcceleration m_alpha0;
    };
}

#endif
