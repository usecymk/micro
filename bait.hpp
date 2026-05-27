#pragma once
#include <raylib.h>
#include <raymath.h>

class Bait
{
public:
    Vector3 position;
    float radius;

    Bait(Vector3 startPos, float r = 0.5f)
    {
        position = startPos;
        radius = r;
    }

    void Draw()
    {
        DrawSphere(position, radius, LIME);
    }

    void Respawn(Vector3 amoebaPos, float spawnDistance)
    {
    
        float angle = (float)GetRandomValue(0, 360) * DEG2RAD;
        
        position.x = amoebaPos.x + std::cos(angle) * spawnDistance;
        position.z = amoebaPos.z + std::sin(angle) * spawnDistance;
        
        position.y = -5.0f + radius; 
    }
};