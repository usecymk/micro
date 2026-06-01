#ifndef MICRO3D_PREY_BEHAVIOR_STATE_MACHINE_H
#define MICRO3D_PREY_BEHAVIOR_STATE_MACHINE_H

#include <algorithm>
#include <cmath>
#include <raylib.h>
#include <raymath.h>

#include "BehaviorStateMachineBase.h"
#include "BehaviorTypes.h"

class PreyBehaviorStateMachine : public BehaviorStateMachineBase {
private:
    // Scale of 0.0 to 100.0 for legacy sensor-style state.
    float fear;
    float hunger;
    float temperatureStress;

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
    PreyBehaviorStateMachine(float fThresh = 50.0f, float hThresh = 50.0f, float tThresh = 30.0f)
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

    bool isScared() const { return currentState == State::SCARED; }
    bool isHungry() const { return currentState == State::HUNGRY; }
    bool isSeekingTemperature() const { return currentState == State::SEEKING_TEMP; }
    bool isIdle() const { return currentState == State::IDLE; }

    float getFear() const { return fear; }
    float getHunger() const { return hunger; }
    float getTemperatureStress() const { return temperatureStress; }

    InternalState state;
    SwimMC        swimMC;
    TurnMC        turnMC;
    Behavior      behavior = Behavior::WANDER;

    void setHeading(Vector3 h, float yaw) { bHeading = h; currentYaw = yaw; }

    void update(float dt)
    {
        state.update(dt);
        if (wallHitCooldown > 0.0f) wallHitCooldown -= dt;
        updateBehavior(dt);
    }

    void onWallHit(Vector3 awayDir, float intensity = 0.25f)
    {
        addBoundaryAvoidance(awayDir, intensity);
        if (wallHitCooldown <= 0.0f)
        {
            targetYaw       = atan2f(awayDir.x, awayDir.z);
            wallHitCooldown = 1.5f;
        }
    }

    void updatePredatorThreat(Vector3 selfPos,
                              Vector3 predatorPos,
                              float alertRadius = 5.0f,
                              float panicRadius = 1.4f)
    {
        Vector3 away = Vector3Subtract(selfPos, predatorPos);
        away.y *= 0.35f;

        float dist = Vector3Length(away);
        if (dist >= alertRadius || dist <= 1e-6f)
            return;

        float threat = 1.0f - Clamp(dist / alertRadius, 0.0f, 1.0f);
        if (dist < panicRadius)
            threat = std::max(threat, 0.65f);

        state.onPredatorNearby(threat);
        fleeDirection = Vector3Normalize(away);
        targetYaw = atan2f(fleeDirection.x, fleeDirection.z);
    }

    void updateVerticalBoundsAvoidance(float y,
                                       float floorY,
                                       float ceilY,
                                       float margin = 0.75f)
    {
        verticalAvoidance = 0.0f;
        boundaryAvoidDirection = {0.0f, 0.0f, 0.0f};
        boundaryAvoidStrength = 0.0f;

        float floorDistance = y - floorY;
        float ceilDistance = ceilY - y;
        if (floorDistance < margin)
        {
            verticalAvoidance = Clamp((margin - floorDistance) / margin, 0.0f, 1.0f);
        }
        else if (ceilDistance < margin)
        {
            verticalAvoidance = -Clamp((margin - ceilDistance) / margin, 0.0f, 1.0f);
        }

        if (std::fabs(verticalAvoidance) > 0.01f)
        {
            addBoundaryAvoidance({0.0f, verticalAvoidance, 0.0f}, std::fabs(verticalAvoidance));
        }
    }

    void updateTemperaturePreference(float currentTemp,
                                     Vector3 gradient,
                                     float preferredTemp = 40.0f,
                                     float comfortableBand = 2.0f)
    {
        float tempError = currentTemp - preferredTemp;
        float absError = std::fabs(tempError);
        temperaturePreferenceStrength = 0.0f;
        temperatureDirection = {0.0f, 0.0f, 0.0f};

        if (absError <= comfortableBand)
            return;

        gradient.y = 0.0f;
        if (Vector3Length(gradient) <= 1e-6f)
            return;

        Vector3 improvingDirection = tempError < 0.0f
            ? Vector3Normalize(gradient)
            : Vector3Normalize(Vector3Negate(gradient));

        temperatureDirection = improvingDirection;
        temperaturePreferenceStrength = Clamp(
            (absError - comfortableBand) / 12.0f,
            0.0f,
            1.0f);
    }

    Vector3 getFleeDirection() const { return fleeDirection; }
    Vector3 getTemperatureDirection() const { return temperatureDirection; }
    Vector3 getBoundaryAvoidDirection() const { return boundaryAvoidDirection; }

    void resetBehavior()
    {
        state           = InternalState{};
        behavior        = Behavior::WANDER;
        swimMC          = SwimMC{};
        turnMC          = TurnMC{};
        targetYaw       = 0.0f;
        wanderTimer     = 0.0f;
        fleeDirection   = {0.0f, 0.0f, 0.0f};
        wallHitCooldown = 0.0f;
        verticalAvoidance = 0.0f;
        boundaryAvoidDirection = {0.0f, 0.0f, 0.0f};
        boundaryAvoidStrength = 0.0f;
        temperatureDirection = {0.0f, 0.0f, 0.0f};
        temperaturePreferenceStrength = 0.0f;
    }

private:
    Vector3 bHeading = {0.0f, 0.0f, -1.0f};
    float currentYaw = 0.0f;
    float targetYaw = 0.0f;
    float wanderTimer = 0.0f;
    Vector3 fleeDirection = {0.0f, 0.0f, 0.0f};
    float wallHitCooldown = 0.0f;
    float verticalAvoidance = 0.0f;
    Vector3 boundaryAvoidDirection = {0.0f, 0.0f, 0.0f};
    float boundaryAvoidStrength = 0.0f;
    Vector3 temperatureDirection = {0.0f, 0.0f, 0.0f};
    float temperaturePreferenceStrength = 0.0f;

    void updateBehavior(float dt)
    {
        if (state.fear   > 0.5f) {
            behavior = Behavior::ESCAPE;
        }
        else if (boundaryAvoidStrength > 0.05f) {
            behavior = Behavior::AVOID_BOUNDARY;
        }
        else if (temperaturePreferenceStrength > 0.05f) {
            behavior = Behavior::SEEK_TEMP;
        }
        // else if (state.hunger > 0.7f) {
        //     behavior = Behavior::SEEK_FOOD;
        // }
        else {
            behavior = Behavior::WANDER;
        }

        turnMC.pitch = verticalAvoidance * 0.8f;

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
            case Behavior::SEEK_TEMP:
                doSeekTemp(dt);
                break;
            case Behavior::AVOID_BOUNDARY:
                doAvoidBoundary(dt);
                break;
        }
    }

    void doWander(float dt)
    {
        swimMC.speed = 0.5f;
        wanderTimer -= dt;
        if (wanderTimer <= 0.0f)
        {
            wanderTimer = 2.0f + nextRandom01() * 3.0f;
            float randTurn = (nextRandom01() * 2.0f - 1.0f) * 1.2f;
            targetYaw = currentYaw + randTurn;
        }
        steerTowardYaw();
    }

    void doSeekFood(float /*dt*/)
    {
        swimMC.speed = 0.75f;
        turnMC.left  = 0.0f;
        turnMC.right = 0.0f;
    }

    void doEscape(float /*dt*/)
    {
        swimMC.speed = 0.9f;
        if (Vector3Length(fleeDirection) > 0.1f) {
            steerTowardYaw();
        }
    }

    void doSeekTemp(float /*dt*/)
    {
        swimMC.speed = 0.45f + 0.25f * temperaturePreferenceStrength;
        if (Vector3Length(temperatureDirection) > 0.1f) {
            targetYaw = atan2f(temperatureDirection.x, temperatureDirection.z);
            steerTowardYaw();
        }
        else {
            turnMC.left = turnMC.right = 0.0f;
        }
    }

    void doAvoidBoundary(float /*dt*/)
    {
        swimMC.speed = 0.45f + 0.15f * boundaryAvoidStrength;
        turnMC.pitch = verticalAvoidance * 0.65f;

        Vector3 horizontal = boundaryAvoidDirection;
        horizontal.y = 0.0f;
        if (Vector3Length(horizontal) > 0.1f)
        {
            targetYaw = atan2f(horizontal.x, horizontal.z);
            steerTowardYaw();
        }
        else
        {
            turnMC.left = turnMC.right = 0.0f;
        }
    }

    void addBoundaryAvoidance(Vector3 awayDir, float strength)
    {
        if (Vector3Length(awayDir) <= 1e-6f)
            return;

        Vector3 weightedAway = Vector3Scale(Vector3Normalize(awayDir), strength);
        boundaryAvoidDirection = Vector3Add(boundaryAvoidDirection, weightedAway);
        if (Vector3Length(boundaryAvoidDirection) > 1e-6f)
            boundaryAvoidDirection = Vector3Normalize(boundaryAvoidDirection);
        boundaryAvoidStrength = std::max(boundaryAvoidStrength, Clamp(strength, 0.0f, 1.0f));
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

#endif // MICRO3D_PREY_BEHAVIOR_STATE_MACHINE_H
