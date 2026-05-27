#ifndef MICRO3D_GRAVITY_FORCE_H
#define MICRO3D_GRAVITY_FORCE_H

#include <raylib.h>
#include <raymath.h>

#include "ForceGenerator.h"
#include "PhysicsBody.h"

class GravityForce : public ForceGenerator
{
    Vector3 g;

public:
    explicit GravityForce(Vector3 gravity = {0.0f, -9.81f, 0.0f}) : g(gravity) {}

    void setGravity(Vector3 gravity) { g = gravity; }

    void apply(PhysicsBody &body, float /*dt*/) override
    {
        for (auto &n : body.getNodes())
            n.force = Vector3Add(n.force, Vector3Scale(g, n.mass));
    }
};

#endif
