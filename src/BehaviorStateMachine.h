#ifndef BEHAVIOR_STATE_MACHINE_H
#define BEHAVIOR_STATE_MACHINE_H

#include <cmath>
#include <cstdlib>
#include <raylib.h>
#include <raymath.h>

//motor controllers for flagella bact
struct SwimMC
{
    float speed = 0.5f;
    float getAmplitude() const { return 0.03f + speed * 0.08f; }
    float getFrequency() const { return 1.5f  + speed * 3.0f;  }
    float getThrust()    const { return speed * getAmplitude() * getFrequency() * 6.0f; }
};

struct TurnMC
{
    float left  = 0.0f;
    float right = 0.0f;
    float pitch = 0.0f;
};


struct InternalState
{
    float hunger = 0.0f;
    float fear = 0.0f;

    enum class CauseOfDeath { NONE, STARVATION, ATTACK };
    bool alive = true;
    CauseOfDeath causeOfDeath = CauseOfDeath::NONE;

    int hitCount = 0;
    float hitWindowTimer = 0.0f;

    void update(float dt)
    {
        if (!alive) {
            return;
        }
        hunger += 0.015f * dt;
        hunger = Clamp(hunger, 0.0f, 1.0f);
        fear *= expf(-1.2f * dt);

        if (hitWindowTimer > 0.0f) {
            hitWindowTimer -= dt;
            if (hitWindowTimer <= 0.0f) {
                hitCount = 0;
            }
        }
        if (hunger >= 1.0f) {
            alive = false;
            causeOfDeath = CauseOfDeath::STARVATION;
        }
    }

    void onEat() { 
        hunger = 0.0f; 
    }
    void feed(float amount) {
        hunger = Clamp(hunger - amount, 0.0f, 1.0f);
    }
    void onPredatorNearby(float prox) { 
        fear = Clamp(fear + prox * 0.6f, 0.0f, 1.0f); 
    }

    void onAttackHit()
    {
        if (hitWindowTimer <= 0.0f) { 
            hitCount = 1; 
            hitWindowTimer = 7.0f; 
        }
        else {
            hitCount++;
        }
        onPredatorNearby(0.8f);
        if (hitCount >= 3) { 
            alive = false; 
            causeOfDeath = CauseOfDeath::ATTACK; 
        }
    }
};


enum class Behavior { WANDER, SEEK_FOOD, ESCAPE };

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
    float getHungerThreshold() const { return hungerThreshold; }
    float getTemperatureThreshold() const { return temperatureThreshold; }

    // ── Organism behavior ─────────────────────────────────────────────────────

    InternalState state;
    SwimMC        swimMC;
    TurnMC        turnMC;
    Behavior      behavior = Behavior::WANDER;

    //called by Bacteria each frame after deriving heading from node positions:
    void setHeading(Vector3 h, float yaw) { bHeading = h; currentYaw = yaw; }

    // Chemotaxis input: `dir` points up the nutrient concentration gradient,
    // `localConcentration` is how rich the water is right here (normalized 0..1).
    void setFoodTarget(Vector3 dir, float localConcentration)
    {
        foodConcentration = localConcentration;
        if (Vector3Length(dir) > 1e-5f)
            foodDirection = Vector3Normalize(dir);
        else
            foodDirection = {0.0f, 0.0f, 0.0f};
    }

    void update(float dt)
    {
        state.update(dt);
        if (wallHitCooldown > 0.0f) wallHitCooldown -= dt;
        updateBehavior(dt);
    }

    void onWallHit(Vector3 awayDir)
    {
        if (wallHitCooldown <= 0.0f)
        {
            targetYaw       = atan2f(awayDir.x, awayDir.z);
            wallHitCooldown = 1.5f;
        }
    }

    void onPredatorNearby(Vector3 awayDir, float proximity)
    {
        if (Vector3Length(awayDir) <= 1e-4f)
            return;

        fleeDirection = Vector3Normalize(awayDir);
        targetYaw = atan2f(fleeDirection.x, fleeDirection.z);
        targetPitch = Clamp(fleeDirection.y, -1.0f, 1.0f);
        state.onPredatorNearby(proximity);
    }

    Vector3 getFleeDirection() const { return fleeDirection; }

    void resetBehavior()
    {
        state           = InternalState{};
        behavior        = Behavior::WANDER;
        swimMC          = SwimMC{};
        turnMC          = TurnMC{};
        targetYaw        = 0.0f;
        wanderTimer      = 0.0f;
        targetPitch      = 0.0f;
        wanderPitchTimer = 0.0f;
        fleeDirection    = {0.0f, 0.0f, 0.0f};
        wallHitCooldown  = 0.0f;
    }

private:
    //organism FSM state
    Vector3 bHeading        = {0.0f, 0.0f, -1.0f};
    float   currentYaw      = 0.0f;
    float   targetYaw       = 0.0f;
    float   wanderTimer     = 0.0f;
    float   targetPitch     = 0.0f;
    float   wanderPitchTimer = 0.0f;
    Vector3 fleeDirection   = {0.0f, 0.0f, 0.0f};
    float   wallHitCooldown = 0.0f;
    Vector3 foodDirection     = {0.0f, 0.0f, 0.0f};
    float   foodConcentration = 0.0f;

    void updateBehavior(float dt)
    {
        if (state.fear   > 0.5f) {
            behavior = Behavior::ESCAPE;
        }
        else if (state.hunger > 0.7f) {
            behavior = Behavior::SEEK_FOOD;
        }
        else {
            behavior = Behavior::WANDER;
        }

        switch (behavior)
        {
            case Behavior::WANDER:
                doWander(dt);   
                break;
            case Behavior::SEEK_FOOD: 
                doSeekFood(dt); 
                break;
            case Behavior::ESCAPE:    
                doEscape(dt);   
                break;
        }
    }

    void doWander(float dt)
    {
        swimMC.speed = 0.7f;
        wanderTimer -= dt;
        if (wanderTimer <= 0.0f)
        {
            wanderTimer = 2.0f + (float)(rand() % 300) / 100.0f;
            float randTurn = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * 1.2f;
            targetYaw = currentYaw + randTurn;
        }
        wanderPitchTimer -= dt;
        if (wanderPitchTimer <= 0.0f)
        {
            wanderPitchTimer = 3.0f + (float)(rand() % 300) / 100.0f;
            targetPitch = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f);
        }
        turnMC.pitch = targetPitch * 0.4f;
        steerTowardYaw();
    }

    void doSeekFood(float dt)
    {
        if (Vector3Length(foodDirection) < 0.1f)
        {
            doWander(dt);
            return;
        }


        swimMC.speed = 0.9f + 0.4f * state.hunger;
        targetYaw   = atan2f(foodDirection.x, foodDirection.z);
        targetPitch = Clamp(foodDirection.y, -1.0f, 1.0f);
        turnMC.pitch = targetPitch * 0.5f;
        steerTowardYaw();
    }

    void doEscape(float /*dt*/)
    {
        swimMC.speed = 1.0f;
        if (Vector3Length(fleeDirection) > 0.1f) {
            turnMC.pitch = targetPitch * 0.6f;
            steerTowardYaw();
        }
    }

    void steerTowardYaw()
    {
        float diff = targetYaw - currentYaw;
        while (diff >  3.14159265f) {
            diff -= 2.0f * 3.14159265f;
        }
        while (diff < -3.14159265f) {
            diff += 2.0f * 3.14159265f;
        }

        const float deadzone = 0.08f;
        if (diff > deadzone) {
            turnMC.left  = fminf(1.0f, diff * 1.5f);
            turnMC.right = 0.0f;
        }
        else if (diff < -deadzone) {
            turnMC.right = fminf(1.0f, -diff * 1.5f);
            turnMC.left  = 0.0f;
        }
        else {
            turnMC.left = turnMC.right = 0.0f;
        }
    }
};

#endif // BEHAVIOR_STATE_MACHINE_H
