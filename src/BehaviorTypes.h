#ifndef MICRO3D_BEHAVIOR_TYPES_H
#define MICRO3D_BEHAVIOR_TYPES_H

#include <cmath>
#include <raylib.h>
#include <raymath.h>

// Motor controllers for flagella bacteria.
struct SwimMC
{
    float speed = 0.5f;
    float getAmplitude() const { return 0.03f + speed * 0.08f; }
    float getFrequency() const { return 1.5f  + speed * 3.0f;  }
    float getThrust()    const { return speed * getAmplitude() * getFrequency() * 2.6f; }
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
    }

    void onEat()
    {
        hunger = 0.0f;
    }

    void onPredatorNearby(float prox)
    {
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

enum class Behavior { WANDER, SEEK_FOOD, ESCAPE, SEEK_TEMP, AVOID_BOUNDARY };

enum class PredatorIntent { AVOID, EAT, SEEK_TEMP, WANDER };

struct PreyCandidate
{
    int id = -1;
    Vector3 position = {0.0f, 0.0f, 0.0f};
    float radius = 0.0f;
    float sizeScore = 0.0f;
    float escapeScore = 0.0f;

    PreyCandidate() = default;
    PreyCandidate(int preyId,
                  Vector3 preyPosition,
                  float preyRadius,
                  float preySizeScore,
                  float preyEscapeScore)
        : id(preyId),
          position(preyPosition),
          radius(preyRadius),
          sizeScore(preySizeScore),
          escapeScore(preyEscapeScore) {}
};

struct PredatorDecision
{
    PredatorIntent intent = PredatorIntent::WANDER;
    Vector3 target = {0.0f, 0.0f, 0.0f};
    int preyId = -1;
    bool hasPrey = false;
    bool captured = false;
    float selectedCost = 0.0f;
    float urgency = 0.0f;
};

#endif // MICRO3D_BEHAVIOR_TYPES_H
