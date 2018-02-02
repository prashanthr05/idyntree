/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/DynamicalSystem.h"
#include "iDynTree/Controller.h"
#include "iDynTree/Core/Utils.h"

namespace iDynTree {
    namespace optimalcontrol {

        DynamicalSystem::DynamicalSystem(size_t stateSpaceSize,
                                         size_t controlSpaceSize)
        : m_stateSize(stateSpaceSize)
        , m_controlSize(controlSpaceSize)
        {}

        DynamicalSystem::~DynamicalSystem() {}

        size_t DynamicalSystem::stateSpaceSize() const{ return m_stateSize; }
        size_t DynamicalSystem::controlSpaceSize() const{ return m_controlSize; }

        bool  DynamicalSystem::dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                            double time,
                                                            MatrixDynSize& dynamicsDerivative)
        { return false; }

        bool DynamicalSystem::dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                             double time,
                                                             MatrixDynSize& dynamicsDerivative)
        { return false; }

//        bool DynamicalSystem::setController(std::shared_ptr<Controller> controllerPointer){
//            if (controllerPointer->controlSpaceSize() != m_controlSize){
//                reportError("DynamicalSystem", "setController", "The controller dimension is not coherent with the controlSpaceSize.");
//                return false;
//            }
//            m_controller_ptr = controllerPointer;
//            return true;
//        }

//        const std::weak_ptr<const Controller> DynamicalSystem::controller() const
//        {
//            return m_controller_ptr;
//        }

    }
}
