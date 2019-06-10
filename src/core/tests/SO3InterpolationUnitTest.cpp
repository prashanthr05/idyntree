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
#include <iDynTree/Core/TestUtils.h>


bool SO3InterpTest()
{
    iDynTree::SO3Interpolation interp;

    iDynTree::AngAcceleration alpha0;
    alpha0.zero();
//     alpha0(0) = 0.5;
//     alpha0(1) = 0.1;
//     alpha0(2) = 0.1;

    iDynTree::AngVelocity omega0;
    omega0.zero();
//     omega0(0) = 0.5;
//     omega0(1) = 0.1;
//     omega0(2) = 0.1;

    iDynTree::VectorDynSize time;
    time.resize(3);
    time(0) = 0;
    time(1) = 1;
    time(2) = 2;

//     iDynTree::Rotation R1 = iDynTree::Rotation::RPY(0, 0, 0);
//     iDynTree::Rotation R2 = iDynTree::Rotation::RPY(0, 1.57, 0);
//     iDynTree::Rotation R3 = iDynTree::Rotation::RPY(0, 3.14, 0);
    iDynTree::Rotation R1 = iDynTree::Rotation::RPY(0, 0, 0);
    iDynTree::Rotation R2 = iDynTree::Rotation::RPY(0, 0, 0);
    iDynTree::Rotation R3 = iDynTree::Rotation::RPY(0, 0, 0);
    std::vector<iDynTree::Rotation> R;
    R.push_back(R1);
    R.push_back(R2);
    R.push_back(R3);

    bool ok = interp.setInitialConditions(omega0, alpha0);
    iDynTree::assertTrue(ok);
    ok = interp.setData(time, R);
    iDynTree::assertTrue(ok);

    iDynTree::Rotation out = interp.evaluatePoint(0.5);
    std::cout << "At time t = 0.5, Rotation is,"   << std::endl << out.toString();
    ok = (iDynTree::toEigen(out).trace() != -1);
    iDynTree::assertTrue(ok);
    return true;
}

int main()
{
    iDynTree::assertTrue(SO3InterpTest());
    return EXIT_SUCCESS;
}

