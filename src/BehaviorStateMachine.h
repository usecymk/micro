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
    float getAmplitude() const { 
        return 0.03f + speed * 0.08f; 
    }
    float getFrequency() const { 
        return 1.5f  + speed * 3.0f;  
    }

    static constexpr float THRUST_SCALE = 0.7f; 
    float getThrust() const { 
        return speed * getAmplitude() * getFrequency() * 6.0f; 
    }
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

    // ── temperature tolerance ──────────────────────────────────────────
    float optimalTemp   = 37.0f;
    float tempTolerance = 12.0f;
    float tempStress    = 0.0f;   // 0 = comfortable, 1 = lethal

    enum class CauseOfDeath { NONE, STARVATION, ATTACK, TEMPERATURE };
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
        if (tempStress >= 1.0f) {
            alive = false;
            causeOfDeath = CauseOfDeath::TEMPERATURE;
        }
    }

    // Call once per frame with ambient temp at the bacterium's position.
    // Climbs while outside [optimalTemp ± tempTolerance], decays back down
    // while comfortable.
    void updateTemperature(float ambientTemp, float dt)
    {
        if (!alive) return;

        float deviation = std::fabs(ambientTemp - optimalTemp) - tempTolerance;
        if (deviation > 0.0f)
        {
            float severity = deviation / std::max(tempTolerance, 1e-3f);
            tempStress = Clamp(tempStress + severity * 0.05f * dt, 0.0f, 1.0f);
        }
        else
        {
            tempStress = Clamp(tempStress - 0.10f * dt, 0.0f, 1.0f);
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

    bool isNearDeath(float hungerThresh = 0.9f, float tempStressThresh = 0.85f) const
    {
        return alive && (hunger >= hungerThresh || tempStress >= tempStressThresh);
    }
};


enum class Behavior { AVOID_OBSTACLE, WANDER, SEEK_FOOD, ESCAPE, SEEK_TEMP };

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
    void setHeadingAndTarget(Vector3 h, float yaw) { bHeading = h; currentYaw = yaw; targetYaw = yaw; }

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

    // gradientTowardWarmer: unit vector pointing toward increasing temperature.
    // currentAmbientTemp / optimalTemp let doSeekTemp() decide whether to swim
    // up or down the gradient (too cold -> follow it, too hot -> reverse it).
    void setTemperatureSensing(Vector3 gradientTowardWarmer, float currentAmbientTemp)
    {
        lastAmbientTemp = currentAmbientTemp;
        if (Vector3Length(gradientTowardWarmer) > 1e-5f)
            tempGradient = Vector3Normalize(gradientTowardWarmer);
        else
            tempGradient = {0.0f, 0.0f, 0.0f};
    }

    void setObstacleSense(Vector3 avoidDir, float urgency)
    {
        obstacleUrgency = Clamp(urgency, 0.0f, 1.0f);
        if (Vector3Length(avoidDir) > 1e-5f)
            obstacleAvoidDir = Vector3Normalize(avoidDir);
        else
            obstacleAvoidDir = {0.0f, 0.0f, 0.0f};
    }

    Vector3 getAvoidDirection() const { return obstacleAvoidDir; }
    bool isAvoidingObstacle() const { return behavior == Behavior::AVOID_OBSTACLE; }

    void update(float dt, bool nearWall = false)
    {
        state.update(dt);
        if (wallHitCooldown > 0.0f) wallHitCooldown -= dt;

        if (nearWall)
        {
            // obstacle check still runs near the wall — hitting an obstacle while
            // hugging the wall shouldn't be ignored
            if (obstacleUrgency > 0.12f)
            {
                if (behavior != Behavior::AVOID_OBSTACLE)
                    savedBehavior = behavior;
                behavior = Behavior::AVOID_OBSTACLE;
                doAvoidObstacle(dt);
                return;
            }
            // otherwise wall-avoidance wins — targetYaw already set by onWallHit()
            steerTowardYaw();
            return;
        }

        updateBehavior(dt);
    }

    void onWallHit(Vector3 awayDir)
    {
        targetYaw = atan2f(awayDir.x, awayDir.z);   // no cooldown gate — keep correcting every frame near wall
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
        fleeDirection      = {0.0f, 0.0f, 0.0f};
        wallHitCooldown    = 0.0f;
        obstacleAvoidDir   = {0.0f, 0.0f, 0.0f};
        obstacleUrgency    = 0.0f;
        savedBehavior      = Behavior::WANDER;
    }

private:
    //organism FSM state
    Vector3 bHeading        = {0.0f, 0.0f, -1.0f};
    float   currentYaw      = 0.0f;
    float   targetYaw       = 0.0f;
    float   wanderTimer     = 0.0f;
    float   targetPitch     = 0.0f;
    float   wanderPitchTimer = 0.0f;
    Vector3 fleeDirection     = {0.0f, 0.0f, 0.0f};
    float   wallHitCooldown   = 0.0f;
    Vector3 foodDirection     = {0.0f, 0.0f, 0.0f};
    float   foodConcentration = 0.0f;
    Vector3 obstacleAvoidDir  = {0.0f, 0.0f, 0.0f};
    float   obstacleUrgency   = 0.0f;
    Behavior savedBehavior    = Behavior::WANDER;
    Vector3 tempGradient      = {0.0f, 0.0f, 0.0f};   // points toward warmer; sign flipped if too hot
    float   lastAmbientTemp   = 20.0f;

    void updateBehavior(float dt)
    {
        if (obstacleUrgency > 0.12f)
        {
            if (behavior != Behavior::AVOID_OBSTACLE)
                savedBehavior = behavior;
            behavior = Behavior::AVOID_OBSTACLE;
            doAvoidObstacle(dt);
            return;
        }

        if (behavior == Behavior::AVOID_OBSTACLE)
            behavior = savedBehavior;

        bool criticalHunger = state.hunger     >= 0.70f;
        bool criticalTemp   = state.tempStress >= 0.85f;

        if (criticalHunger || criticalTemp)
        {
            // both override fear; hunger always takes priority over temp
            behavior = criticalHunger ? Behavior::SEEK_FOOD : Behavior::SEEK_TEMP;
        }
        else if (state.fear > 0.35f)
        {
            behavior = Behavior::ESCAPE;
        }
        else if (state.hunger > 0.5f)
        {
            behavior = Behavior::SEEK_FOOD;
        }
        else if (state.tempStress > 0.5f)
        {
            behavior = Behavior::SEEK_TEMP;
        }
        else
        {
            behavior = Behavior::WANDER;
        }

        switch (behavior)
        {
            case Behavior::AVOID_OBSTACLE: doAvoidObstacle(dt); break;
            case Behavior::WANDER:         doWander(dt);        break;
            case Behavior::SEEK_FOOD:      doSeekFood(dt);      break;
            case Behavior::ESCAPE:         doEscape(dt);        break;
            case Behavior::SEEK_TEMP:      doSeekTemp(dt);      break;
        }
    }

    void doAvoidObstacle(float /*dt*/)
    {
        swimMC.speed = 0.88f + obstacleUrgency * 0.56f;
        if (Vector3Length(obstacleAvoidDir) > 0.1f)
        {
            targetYaw   = atan2f(obstacleAvoidDir.x, obstacleAvoidDir.z);
            targetPitch = Clamp(obstacleAvoidDir.y, -1.0f, 1.0f);
            turnMC.pitch = targetPitch * 0.75f;
            steerTowardYaw();
        }
    }

    void doWander(float dt)
    {
        swimMC.speed = 0.56f;
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


        swimMC.speed = 0.72f + 0.32f * state.hunger;
        targetYaw   = atan2f(foodDirection.x, foodDirection.z);
        targetPitch = Clamp(foodDirection.y, -1.0f, 1.0f);
        turnMC.pitch = targetPitch * 0.5f;
        steerTowardYaw();
    }

    void doEscape(float /*dt*/)
    {
        swimMC.speed = 1.2f;
        if (Vector3Length(fleeDirection) > 0.1f) {
            turnMC.pitch = targetPitch * 0.6f;
            steerTowardYaw();
        }
    }

    void doSeekTemp(float /*dt*/)
    {
        if (Vector3Length(tempGradient) < 0.1f)
        {
            doWander(0.0f);
            return;
        }

        // too cold -> swim toward warmer (follow gradient); too hot -> swim away
        Vector3 dir = (lastAmbientTemp < state.optimalTemp)
            ? tempGradient
            : Vector3Negate(tempGradient);

        swimMC.speed = 0.68f;
        targetYaw    = atan2f(dir.x, dir.z);
        targetPitch  = Clamp(dir.y, -1.0f, 1.0f);
        turnMC.pitch = targetPitch * 0.5f;
        steerTowardYaw();
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
