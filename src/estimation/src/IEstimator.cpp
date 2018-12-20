#include <iDynTree/Estimation/IEstimator.h>


namespace iDynTree
{   

/** 
 * begin SavedEstimatorState class methods
 */
SavedEstimatorState::SavedEstimatorState(const IEstimatorInternalState& state) : _state(state)
{    
}
/** 
 * end SavedEstimatorState class methods
 */


/** 
 * begin IEstimatorInternalStateManager class methods
 */

IEstimatorInternalStateManager::IEstimatorInternalStateManager() : _lastEstimatorState(nullptr), m_verbose(true)
{
}
        
IEstimatorInternalStateManager::~IEstimatorInternalStateManager()
{
    for (std::unique_ptr<SavedEstimatorState>& elem : _savedEstimatorStateBuffer)
    {
        elem.reset();
    }
    _savedEstimatorStateBuffer.clear();
    
    if (_lastEstimatorState != nullptr)
    {
        _lastEstimatorState.reset();
    }
}
        
void IEstimatorInternalStateManager::saveEstimatorStateInBuffer(std::unique_ptr<SavedEstimatorState>& state)
{
    _savedEstimatorStateBuffer.push_back(std::move(state));
}

std::unique_ptr<SavedEstimatorState> IEstimatorInternalStateManager::getLastEstimatorStateAndResetBuffer()
{
    _lastEstimatorState = std::move(_savedEstimatorStateBuffer.back());    
    _savedEstimatorStateBuffer.pop_back();
    if (m_verbose)
    {
        std::cout << "IEstimatorInternalStateManager: Resetting the saved state buffer to " << _savedEstimatorStateBuffer.size() << " states." << std::endl;
        std::cout << "IEstimatorInternalStateManager: The latest time index in the saved state buffer is [" <<  _savedEstimatorStateBuffer.back()->getTimeIdx() << "]" << std::endl;
    }
    if (!_lastEstimatorState)
    {
        throw std::runtime_error("ERROR: IEstimatorInternalStateManager: No saved states in the buffer");
        return {};    
    }
    return std::move(_lastEstimatorState);
}

std::unique_ptr<SavedEstimatorState> IEstimatorInternalStateManager::getEstimatorStateAtTimeIndexAndResetBuffer(unsigned int timeIdx)
{
     if (timeIdx > _savedEstimatorStateBuffer.back()->getTimeIdx())
     {
         throw std::runtime_error("ERRROR: IEstimatorInternalStateManager: Failed trying to access a state from future time. No state with such time index available in the buffer");
     }
     
    // TODO: replace this linear search with an efficient search 
    unsigned int saveID = 0;
    for (unsigned int ID = 0; ID < _savedEstimatorStateBuffer.size(); ID++)
    {
        if (timeIdx == _savedEstimatorStateBuffer[ID]->getTimeIdx())
        {
            saveID = ID;
            break;
        }
    }
    _lastEstimatorState = std::move(_savedEstimatorStateBuffer.at(saveID));    
    _savedEstimatorStateBuffer.erase(_savedEstimatorStateBuffer.begin()+saveID, _savedEstimatorStateBuffer.end());
    if (m_verbose)
    {
        std::cout << "IEstimatorInternalStateManager: Resetting the saved state buffer to " << _savedEstimatorStateBuffer.size() << " states." << std::endl;
        std::cout << "IEstimatorInternalStateManager: The latest time index in the saved state buffer is [" <<  _savedEstimatorStateBuffer.back()->getTimeIdx() << "]" << std::endl;
    }
    return std::move(_lastEstimatorState);
}


/** 
 * end IEstimatorInternalStateManager class methods
 */


/** 
 * begin IEstimator class methods
 */

IEstimator::IEstimator() : m_reset_state_to_desired(false)
{
    m_saveManager = std::unique_ptr<IEstimatorInternalStateManager>(new IEstimatorInternalStateManager());
}

IEstimator::IEstimator(const IEstimatorInternalState& state)
{
    m_presentEstimatorState = state;
    m_reset_state_to_desired = true;
    m_saveManager = std::unique_ptr<IEstimatorInternalStateManager>(new IEstimatorInternalStateManager());
    //TODO: resetting the states to be handled in a clean way and turn the flag reset_state_to_desired to false
}

        
void IEstimator::saveEstimatorState()
{
    std::unique_ptr<SavedEstimatorState> save = std::make_unique<SavedEstimatorState>(SavedEstimatorState(m_presentEstimatorState));
    m_saveManager->saveEstimatorStateInBuffer(save);
}
        
void IEstimator::loadEstimatorState(const SavedEstimatorState& estimatorState)
{
    m_tempEstimatorState = estimatorState._state;
    m_reset_state_to_desired = true;
    //TODO: resetting the states to be handled in a clean way and turn the flag reset_state_to_desired to false
}

void IEstimator::loadLastSavedEstimatorStateAndResetSaveBuffer()
{
    std::unique_ptr<iDynTree::SavedEstimatorState> savedStateFromManager = m_saveManager->getLastEstimatorStateAndResetBuffer();
    loadEstimatorState(*savedStateFromManager);
}

void IEstimator::loadSavedEstimatorStateAtTimeIndexAndResetSaveBuffer(unsigned int timeIdx)
{
    std::unique_ptr<iDynTree::SavedEstimatorState> savedStateFromManager = m_saveManager->getEstimatorStateAtTimeIndexAndResetBuffer(timeIdx);
    loadEstimatorState(*savedStateFromManager);
}

unsigned int IEstimator::getNrOfSavedStates()
{
    return m_saveManager->getNrOfInternalStatesInBuffer();
}

void IEstimator::setInternalStateManagerVerbose(bool verbose)
{
    m_saveManager->setVerbose(verbose);
}


/** 
 * end IEstimator class methods
 */

    
}
