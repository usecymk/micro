#ifndef MICRO3D_BAIT_H
#define MICRO3D_BAIT_H

#include <cmath>

#include <raylib.h>
#include <raymath.h>

class Bait
{
public:
    Vector3 position;
    float   radius;
    Color   color = LIME;

    Bait(Vector3 startPos, float r = 0.5f)
        : position(startPos), radius(r) {}

    void draw() const
    {
        DrawSphere(position, radius, color);
    }

    void respawn(Vector3 hunterPos, float spawnDistance, float restY = 0.0f)
    {
        float angle = (float)GetRandomValue(0, 360) * DEG2RAD;
        position.x = hunterPos.x + std::cos(angle) * spawnDistance;
        position.z = hunterPos.z + std::sin(angle) * spawnDistance;
        position.y = restY + radius;
    }
};

#endif
