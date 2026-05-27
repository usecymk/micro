#ifndef MICRO3D_DRAG_FORCE_H
#define MICRO3D_DRAG_FORCE_H

#include <raylib.h>
#include <raymath.h>

#include "ForceGenerator.h"
#include "PhysicsBody.h"

// Linear viscous drag: F = -c * m * v on each node.
class DragForce : public ForceGenerator
{
    float coefficient;

public:
    explicit DragForce(float c = 1.8f) : coefficient(c) {}

    void setCoefficient(float c) { coefficient = c; }

    void apply(PhysicsBody &body, float /*dt*/) override
    {
        for (auto &n : body.getNodes())
        {
            Vector3 F = Vector3Scale(n.velocity, -coefficient * n.mass);
            n.force = Vector3Add(n.force, F);
        }
    }
};

#endif
