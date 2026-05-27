#ifndef MICRO3D_BUOYANCY_FORCE_H
#define MICRO3D_BUOYANCY_FORCE_H

#include <algorithm>

#include <raylib.h>
#include <raymath.h>

#include "ForceGenerator.h"
#include "PhysicsBody.h"
#include "FluidEnvironment.h"

class BuoyancyForce : public ForceGenerator
{
    const FluidEnvironment *fluid;
    float smoothing;

public:
    explicit BuoyancyForce(const FluidEnvironment *f, float smoothing_m = 0.1f)
        : fluid(f), smoothing(smoothing_m) {}

    void setSmoothing(float s) { smoothing = s; }

    void apply(PhysicsBody &body, float /*dt*/) override
    {
        if (fluid == nullptr)
            return;

        const float halfBand = 0.5f * std::max(smoothing, 1e-6f);

        for (auto &n : body.getNodes())
        {
            float depth = fluid->surfaceY - n.position.y;
            float t = (depth + halfBand) / (2.0f * halfBand);
            if (t <= 0.0f)
                continue;
            if (t > 1.0f)
                t = 1.0f;

            // F = -rho * V * g 
            Vector3 F = Vector3Scale(fluid->gravity, -fluid->density * n.volume * t);
            n.force = Vector3Add(n.force, F);
        }
    }
};

#endif
