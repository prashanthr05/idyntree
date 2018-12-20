#ifndef IESTIMATOR_H
#define IESTIMATOR_H

#include <deque>
#include <iostream>
#include <memory>

namespace iDynTree
{
    class IEstimator;
    class IEstimatorInternalStateManager;
    class SavedEstimatorState;

    /**
     * @brief struct containting the internal state of the Estimator
     * 
     */
    struct IEstimatorInternalState
    {
        int timeIdx;
        double time;
        // container for iterators
        // container for states
        // container for inputs
        // container for internal status
        // container for parameters
        // container for identifiers
    };
    
    
    
    /**
     * @brief SavedEstimatorState class
     * 
     * Memento class for saving a snapshot of estimator state
     * Accessible only by the IEstimator class
     */
    class SavedEstimatorState
    {            
        friend class IEstimator;
        IEstimatorInternalState _state;        
        SavedEstimatorState(const IEstimatorInternalState& state);
    public:
        unsigned int getTimeIdx() { return _state.timeIdx; }
    };
    
    /**
     * @brief IEstimatorInternalStateManager class
     * Caretaker - Manager of a buffer of saved states 
     */
    class IEstimatorInternalStateManager
    {
        // TODO Add clean up methods to prevent RAM takeover
        // implement max buffer memory 
        
        std::deque<std::unique_ptr<SavedEstimatorState > > _savedEstimatorStateBuffer;
        std::unique_ptr<SavedEstimatorState> _lastEstimatorState;     
        bool m_verbose;
    public:
        IEstimatorInternalStateManager();        
        ~IEstimatorInternalStateManager();
        
        void saveEstimatorStateInBuffer(std::unique_ptr<SavedEstimatorState>& state);
       
        std::unique_ptr<SavedEstimatorState> getEstimatorStateAtTimeIndexAndResetBuffer(unsigned int timeIdx);        
        std::unique_ptr<SavedEstimatorState> getLastEstimatorStateAndResetBuffer(); 
        unsigned int getNrOfInternalStatesInBuffer() { return _savedEstimatorStateBuffer.size(); }
        
        void setVerbose(bool verbose = true) { m_verbose = verbose; }
    };
    
    
    /**
     * @brief Estimator class
     * 
     * Originator of SavedEstimatorState memento object
     */
    class IEstimator
    {
        IEstimatorInternalState m_previousEstimatorState;        
        std::unique_ptr<IEstimatorInternalStateManager> m_saveManager;
        bool m_reset_state_to_desired = false;
        
        void loadEstimatorState(const SavedEstimatorState& estimatorState);        
    public:
        IEstimatorInternalState m_presentEstimatorState;
        IEstimatorInternalState m_tempEstimatorState;        
        IEstimator();
        IEstimator(const IEstimatorInternalState& state);        
 
        // InternalStateManager related methods
        void saveEstimatorState();
        void loadLastSavedEstimatorStateAndResetSaveBuffer();
        void loadSavedEstimatorStateAtTimeIndexAndResetSaveBuffer(unsigned int timeIdx);
        unsigned int getNrOfSavedStates();
        void setInternalStateManagerVerbose(bool verbose = true);
        
        // Estimation related methods
        // initialize the estimator
        virtual bool init() { return true; };
        
        // estimator loop to be called within a thread instantiation
        virtual bool run() { return true; };
        
        // the condition for which the run() loop keeps running
        virtual bool runningCondition() { return true; };
        
        // other possible functions
        // reset estimator to the initial state
        virtual bool reset() { return true; };
        
        // pause the estimator
        virtual bool pause() { return true; };
        
        // check if the estimator is stopped and released
        virtual bool isStopped() { return true; };
        
        // check if the estimator is paused
        virtual bool isPaused() { return true; };
        
        // resume the estimator loop from pause state
        virtual bool resume() { return true; };
        
        // stop and release the estimator
        virtual bool release() { return true; };
        
        // predict step - process dynamics 
        virtual bool predict() { return true; };
        
        // update step - correct with sensor measurements
        virtual bool update() { return true; };
        
        virtual bool setInternalState() { return true; };
        virtual bool getInternalState() { return true; };
    };
        
}

#endif
