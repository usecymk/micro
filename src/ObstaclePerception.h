#ifndef MICRO3D_OBSTACLE_PERCEPTION_H
#define MICRO3D_OBSTACLE_PERCEPTION_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>

#include "Obstacle.h"

enum class AttentionMode
{
    WANDER,
    HUNGRY,
    FEARFUL
};

struct ObstacleSenseParams
{
    float senseRadius      = 3.0f;
    float attentionRadius  = 1.6f;
    float criticalRadius   = 0.4f;
    float attentionWeight  = 1.0f;
};

struct ObstacleSenseResult
{
    bool    detected         = false;
    float   urgency          = 0.0f;
    Vector3 avoidDirection   = {0.0f, 0.0f, 0.0f};
    float   nearestClearance = 1e9f;
};

inline void applySelectiveAttention(ObstacleSenseParams &params, AttentionMode mode)
{
    switch (mode)
    {
        case AttentionMode::FEARFUL:
            params.senseRadius     *= 0.75f;
            params.attentionRadius *= 0.65f;
            params.attentionWeight  = 0.4f;
            break;
        case AttentionMode::HUNGRY:
            params.senseRadius     *= 0.85f;
            params.attentionRadius *= 0.75f;
            params.attentionWeight  = 0.55f;
            break;
        case AttentionMode::WANDER:
        default:
            params.senseRadius     *= 0.9f;
            params.attentionWeight  = 1.0f;
            break;
    }
}

inline ObstacleSenseResult senseObstacles(Vector3 pos,
                                          Vector3 heading,
                                          const std::vector<Obstacle> &obstacles,
                                          ObstacleSenseParams params,
                                          float bodyPadding = 0.0f,
                                          AttentionMode mode = AttentionMode::WANDER)
{
    ObstacleSenseResult result;
    Vector3 repulsion = Vector3Zero();
    float maxUrgency = 0.0f;

    Vector3 headingNorm = Vector3Length(heading) > 1e-5f
        ? Vector3Normalize(heading)
        : Vector3{0.0f, 0.0f, -1.0f};

    for (const auto &obs : obstacles)
    {
        float clearance = obs.signedDistanceToSurface(pos) - bodyPadding;
        if (clearance > params.senseRadius)
            continue;

        result.nearestClearance = std::min(result.nearestClearance, clearance);

        if (mode == AttentionMode::WANDER && clearance > params.attentionRadius)
        {
            Vector3 toCenter = Vector3Subtract(obs.getCenter(), pos);
            float centerDist = Vector3Length(toCenter);
            if (centerDist > 1e-5f)
            {
                float ahead = Vector3DotProduct(headingNorm, Vector3Scale(toCenter, 1.0f / centerDist));
                if (ahead < 0.25f)
                    continue;
            }
        }

        Vector3 surface = obs.closestPointOnSurface(pos);
        Vector3 away = Vector3Subtract(pos, surface);
        float awayLen = Vector3Length(away);
        if (awayLen < 1e-5f)
            away = {1.0f, 0.0f, 0.0f};
        else
            away = Vector3Scale(away, 1.0f / awayLen);

        float localUrgency = 0.0f;
        if (clearance <= params.criticalRadius)
        {
            localUrgency = 1.0f;
        }
        else
        {
            float span = std::max(params.attentionRadius - params.criticalRadius, 1e-4f);
            float t = 1.0f - (clearance - params.criticalRadius) / span;
            localUrgency = Clamp(t, 0.0f, 1.0f) * params.attentionWeight;
        }

        Vector3 towardSurface = Vector3Negate(away);
        float ahead = Vector3DotProduct(headingNorm, towardSurface);
        if (ahead > 0.0f)
            localUrgency = Clamp(localUrgency * (1.0f + ahead * 0.85f), 0.0f, 1.0f);

        if (localUrgency <= 0.02f)
            continue;

        float weight = localUrgency * localUrgency;
        repulsion = Vector3Add(repulsion, Vector3Scale(away, weight));
        maxUrgency = std::max(maxUrgency, localUrgency);
    }

    if (maxUrgency > 0.05f && Vector3Length(repulsion) > 1e-5f)
    {
        result.detected       = true;
        result.urgency        = maxUrgency;
        result.avoidDirection = Vector3Normalize(repulsion);
    }

    return result;
}

#endif
