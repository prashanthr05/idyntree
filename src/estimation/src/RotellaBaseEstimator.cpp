/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/RotellaBaseEstimator.h>

bool iDynTree::RotellaBaseEstimator::ekf_f(const iDynTree::VectorDynSize& x_k,
                                                                 const iDynTree::VectorDynSize& u_k,
                                                                 iDynTree::VectorDynSize& xhat_k_plus_one)
{
    return true;
}

