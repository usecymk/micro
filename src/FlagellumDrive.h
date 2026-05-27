#ifndef MICRO3D_FLAGELLUM_DRIVE_H
#define MICRO3D_FLAGELLUM_DRIVE_H

#include <cmath>

#include "ForceGenerator.h"
#include "PhysicsBody.h"

class FlagellumDrive : public ForceGenerator
{
    int startIndex;
    int count;
    float freq;
    float amp;
    float phaseStep;
    float time = 0.0f;

public:
    FlagellumDrive(int start,
                   int n,
                   float frequency = 3.0f,
                   float amplitude = 1.5f,
                   float phase = 0.55f)
        : startIndex(start), count(n), freq(frequency), amp(amplitude), phaseStep(phase) {}

    void apply(PhysicsBody &body, float dt) override
    {
        if (count < 2)
            return;

        time += dt;
        auto &nodes = body.getNodes();
        for (int i = 0; i < count; i++)
        {
            float t = (float)i / (float)(count - 1);
            float f = amp * t * std::sin(freq * time - i * phaseStep);
            nodes[startIndex + i].force.x += f;
        }
    }
};

#endif
