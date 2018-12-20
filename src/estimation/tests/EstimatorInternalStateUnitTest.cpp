/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/Estimation/IEstimator.h"
#include <cstdio>
#include <cstdlib>

void testEstimatorInternalState()
{
    iDynTree::IEstimatorInternalState state;
    
    state.timeIdx = 0;
    state.time = 0.0;
    
    iDynTree::IEstimator estimator = iDynTree::IEstimator(state);
    std::cout << "state after constructing estimator: " << estimator.m_presentEstimatorState.timeIdx << std::endl;
    
    estimator.saveEstimatorState();
    
    // change state
    estimator.m_presentEstimatorState.timeIdx = 1;
    estimator.m_presentEstimatorState.time = 1.0;
    
    estimator.saveEstimatorState();
    
    std::cout << "nr. of states in buffer: " << estimator.getNrOfSavedStates() << std::endl; 
    
    estimator.m_tempEstimatorState.timeIdx = 10;
    std::cout << "temp state before loading state: " << estimator.m_tempEstimatorState.timeIdx << std::endl;
        
    estimator.loadLastSavedEstimatorStateAndResetSaveBuffer();
    std::cout << "temp state after loading state: " << estimator.m_tempEstimatorState.timeIdx << std::endl;
    
}

int main(int argc, char** argv)
{
    testEstimatorInternalState();
    
    return EXIT_SUCCESS;
}