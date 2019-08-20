/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <vector>

namespace iDynTree
{
    class DiscreteKalmanFilterHelper
    {
    public:
        DiscreteKalmanFilterHelper();
        bool constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                const iDynTree::MatrixDynSize& B,
                                const iDynTree::MatrixDynSize& C,
                                const iDynTree::MatrixDynSize& D);
        bool constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                const iDynTree::MatrixDynSize& B,
                                const iDynTree::MatrixDynSize& C);
        bool constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                const iDynTree::MatrixDynSize& C);

        bool kfSetInitialState(const iDynTree::VectorDynSize& x0);
        bool kfSetStateCovariance(const iDynTree::MatrixDynSize& P);
        bool kfSetSystemNoiseCovariance(const iDynTree::MatrixDynSize& Q);
        bool kfSetMeasurementNoiseCovariance(const iDynTree::MatrixDynSize& R);

        bool kfInit();

        bool kfSetInputVector(const iDynTree::VectorDynSize& u);
        bool kfPredict();

        bool kfSetMeasurementVector(const iDynTree::VectorDynSize& y);
        bool kfUpdate();

        bool kfGetStates(iDynTree::VectorDynSize &x);
        bool kfGetStateCovariance(iDynTree::MatrixDynSize &P);

        bool kfReset();
        bool kfReset(const iDynTree::VectorDynSize& x0, const iDynTree::MatrixDynSize& P0,
                     const iDynTree::MatrixDynSize& Q, const iDynTree::MatrixDynSize& R);

    private:
        size_t m_dim_X;                                ///< state dimension
        size_t m_dim_Y;                                ///< output dimenstion
        size_t m_dim_U;

        iDynTree::VectorDynSize m_x;                   ///< state at time instant k
        iDynTree::VectorDynSize m_x0;
        iDynTree::VectorDynSize m_u;                   ///< input at time instant k
        iDynTree::VectorDynSize m_y;                   ///< measurements at time instant k

        iDynTree::MatrixDynSize m_A;
        iDynTree::MatrixDynSize m_B;
        iDynTree::MatrixDynSize m_C;
        iDynTree::MatrixDynSize m_D;

        iDynTree::MatrixDynSize m_P;
        iDynTree::MatrixDynSize m_P0;
        iDynTree::MatrixDynSize m_Q;
        iDynTree::MatrixDynSize m_R;

        bool m_is_initialized{false};

        bool m_filter_constructed{false};
        bool m_initial_state_set{false};
        bool m_initial_state_covariance_set{false};
        bool m_measurement_noise_covariance_matrix_set{false};
        bool m_system_noise_covariance_matrix_set{false};

        bool m_measurement_updated{false};
        bool m_input_updated{false};

        bool m_use_feed_through{false};
        bool m_use_control_input{false};
    };
}

#endif
