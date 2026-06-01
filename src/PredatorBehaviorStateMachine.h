#ifndef MICRO3D_PREDATOR_BEHAVIOR_STATE_MACHINE_H
#define MICRO3D_PREDATOR_BEHAVIOR_STATE_MACHINE_H

#include <cmath>
#include <limits>
#include <vector>
#include <raylib.h>
#include <raymath.h>

#include "BehaviorStateMachineBase.h"
#include "BehaviorTypes.h"

class PredatorBehaviorStateMachine : public BehaviorStateMachineBase {
public:
    PredatorDecision updatePredator(float dt,
                                    Vector3 predatorPos,
                                    const std::vector<PreyCandidate> &prey,
                                    float arenaRadius,
                                    float currentTemp = 40.0f,
                                    Vector3 tempGradient = {0.0f, 0.0f, 0.0f},
                                    float preferredTemp = 40.0f,
                                    float comfortableBand = 2.0f)
    {
        hunger = Clamp(hunger + hungerRiseRate * dt, 0.0f, 1.0f);

        PredatorDecision decision;
        if (shouldPredatorAvoid(predatorPos, arenaRadius))
        {
            decision.intent = PredatorIntent::AVOID;
            decision.target = predatorAvoidTarget(predatorPos);
            decision.urgency = hunger;
            predatorWanderTimer = 0.0f;
            return decision;
        }

        if (hunger < huntThreshold)
        {
            return satiatedDecision(dt, predatorPos, arenaRadius, currentTemp,
                                    tempGradient, preferredTemp, comfortableBand);
        }

        int bestIndex = -1;
        float bestCost = std::numeric_limits<float>::max();
        float huntUrgency = Clamp((hunger - huntThreshold) / (1.0f - huntThreshold), 0.0f, 1.0f);
        for (int i = 0; i < (int)prey.size(); i++)
        {
            float cost = preyCost(predatorPos, prey[i], huntUrgency);
            if (cost < bestCost)
            {
                bestCost = cost;
                bestIndex = i;
            }
        }

        if (bestIndex >= 0)
        {
            const PreyCandidate &selected = prey[bestIndex];
            decision.intent = PredatorIntent::EAT;
            decision.target = selected.position;
            decision.preyId = selected.id;
            decision.hasPrey = true;
            decision.selectedCost = bestCost;
            decision.captured = predatorCaptured(predatorPos, selected);
            decision.urgency = huntUrgency;
            predatorWanderTimer = 0.0f;
            return decision;
        }

        decision.intent = PredatorIntent::WANDER;
        decision.target = predatorWanderTarget(dt, predatorPos, arenaRadius);
        decision.urgency = huntUrgency;
        return decision;
    }

    void onConsumePrey()
    {
        hunger = Clamp(hunger - satiationGain, 0.0f, 1.0f);
    }

    float getHunger() const { return hunger; }

private:
    float hunger = 0.35f;
    float huntThreshold = 0.45f;
    float hungerRiseRate = 0.018f;
    float satiationGain = 0.75f;
    float predatorSenseRadius = 12.0f;
    float predatorCaptureRadius = 0.75f;
    float predatorCaptureMargin = 0.20f;
    float predatorAvoidMargin = 1.0f;
    float predatorLambdaSize = 0.6f;
    float predatorLambdaEscape = 0.8f;
    float predatorWanderTimer = 0.0f;
    Vector3 predatorWanderPoint = {0.0f, 0.0f, 0.0f};

    static float horizontalDistance(Vector3 a, Vector3 b)
    {
        Vector3 d = Vector3Subtract(b, a);
        d.y = 0.0f;
        return Vector3Length(d);
    }

    bool shouldPredatorAvoid(Vector3 predatorPos, float arenaRadius) const
    {
        float r = std::sqrt(predatorPos.x * predatorPos.x + predatorPos.z * predatorPos.z);
        return r > arenaRadius - predatorAvoidMargin;
    }

    Vector3 predatorAvoidTarget(Vector3 predatorPos) const
    {
        return {0.0f, predatorPos.y, 0.0f};
    }

    PredatorDecision satiatedDecision(float dt,
                                      Vector3 predatorPos,
                                      float arenaRadius,
                                      float currentTemp,
                                      Vector3 tempGradient,
                                      float preferredTemp,
                                      float comfortableBand)
    {
        PredatorDecision decision;
        float tempError = currentTemp - preferredTemp;
        if (std::fabs(tempError) > comfortableBand)
        {
            tempGradient.y *= 0.35f;
            if (Vector3Length(tempGradient) > 1e-6f)
            {
                Vector3 direction = tempError < 0.0f
                    ? Vector3Normalize(tempGradient)
                    : Vector3Normalize(Vector3Negate(tempGradient));
                decision.intent = PredatorIntent::SEEK_TEMP;
                decision.target = Vector3Add(predatorPos, Vector3Scale(direction, 3.0f));
                decision.urgency = hunger;
                predatorWanderTimer = 0.0f;
                return decision;
            }
        }

        decision.intent = PredatorIntent::WANDER;
        decision.target = predatorWanderTarget(dt, predatorPos, arenaRadius);
        decision.urgency = hunger;
        return decision;
    }

    float preyCost(Vector3 predatorPos, const PreyCandidate &prey, float huntUrgency) const
    {
        float d = horizontalDistance(predatorPos, prey.position);
        float effectiveSenseRadius = predatorSenseRadius * (0.65f + 0.55f * huntUrgency);
        if (d > effectiveSenseRadius)
            return std::numeric_limits<float>::max();

        return d * (1.0f
            + predatorLambdaSize * prey.sizeScore
            + predatorLambdaEscape * prey.escapeScore)
            / (0.5f + huntUrgency);
    }

    bool predatorCaptured(Vector3 predatorPos, const PreyCandidate &prey) const
    {
        float captureDistance = predatorCaptureRadius + prey.radius + predatorCaptureMargin;
        return Vector3Distance(predatorPos, prey.position) <= captureDistance;
    }

    Vector3 predatorWanderTarget(float dt, Vector3 predatorPos, float arenaRadius)
    {
        predatorWanderTimer -= dt;
        if (predatorWanderTimer <= 0.0f)
        {
            predatorWanderTimer = 2.0f + nextRandom01() * 3.0f;
            float angle = nextRandom01() * 2.0f * PI;
            float step = 2.5f;
            predatorWanderPoint = {
                predatorPos.x + cosf(angle) * step,
                predatorPos.y,
                predatorPos.z + sinf(angle) * step
            };

            float r = std::sqrt(predatorWanderPoint.x * predatorWanderPoint.x
                              + predatorWanderPoint.z * predatorWanderPoint.z);
            float maxR = arenaRadius - predatorAvoidMargin;
            if (r > maxR && r > 1e-4f)
            {
                float scale = maxR / r;
                predatorWanderPoint.x *= scale;
                predatorWanderPoint.z *= scale;
            }
        }

        return predatorWanderPoint;
    }
};

#endif // MICRO3D_PREDATOR_BEHAVIOR_STATE_MACHINE_H
