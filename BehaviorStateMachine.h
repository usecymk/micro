#ifndef BEHAVIOR_STATE_MACHINE_H
#define BEHAVIOR_STATE_MACHINE_H

class BehaviorStateMachine {
private:
    //scale of 0.0 to 100.0
    float fear;
    float hunger;
    float temperatureStress;

    //thresholds required to trigger each state
    float fearThreshold;
    float hungerThreshold;
    float temperatureThreshold;

    enum class State 
    {
        IDLE,
        SCARED, 
        HUNGRY,
        SEEKING_TEMP
    };

    State currentState;

    void evaluateState() 
    {
        if (fear >= fearThreshold) 
        {
            currentState = State::SCARED;
        } 
        else if (hunger >= hungerThreshold) 
        {
            currentState = State::HUNGRY;
        } 
        else if (temperatureStress >= temperatureThreshold) 
        {
            currentState = State::SEEKING_TEMP;
        } 
        else 
        {
            currentState = State::IDLE;
        }
    }

public:
    BehaviorStateMachine(float fThresh = 50.0f, float hThresh = 50.0f, float tThresh = 30.0f)
        : fear(0.0f), 
          hunger(0.0f), 
          temperatureStress(0.0f),
          fearThreshold(fThresh), 
          hungerThreshold(hThresh), 
          temperatureThreshold(tThresh),
          currentState(State::IDLE) {}

    void updateSensors(float currentFear, float currentHunger, float currentTempStress) 
    {
        fear = currentFear;
        hunger = currentHunger;
        temperatureStress = currentTempStress;
        
        evaluateState();
    }

    // --- State Getters ---
    //only one of these will ever be true at any given time.

    bool isScared() const { 
        return currentState == State::SCARED; 
    }
    
    bool isHungry() const { 
        return currentState == State::HUNGRY; 
    }
    
    bool isSeekingTemperature() const { 
        return currentState == State::SEEKING_TEMP; 
    }

    bool isIdle() const {
        return currentState == State::IDLE;
    }

    // --- Variable Getters ---
    float getFear() const { return fear; }
    float getHunger() const { return hunger; }
    float getTemperatureStress() const { return temperatureStress; }
};

#endif // BEHAVIOR_STATE_MACHINE_H