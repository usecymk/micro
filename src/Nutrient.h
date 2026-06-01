#ifndef MICRO3D_NUTRIENT_H
#define MICRO3D_NUTRIENT_H

#include <algorithm>
#include <cmath>

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"

// Soft spherical nutrient pellet: center + shell nodes, springs, fluid forces.
class Nutrient : public PhysicsBody
{
public:
    static constexpr int SHELL_NODES = 8;

    float radius = 0.35f;
    Color color  = LIME;
    bool  active = true;

    Nutrient() = default;

    float getRadius() const { return radius; }

    Vector3 getCenterPosition() const
    {
        if (nodes.empty()) return Vector3Zero();
        return nodes[0].position;
    }

    void draw() const
    {
        if (!active || nodes.empty()) return;

        float shellR = radius;
        for (int i = 1; i < (int)nodes.size(); i++)
            shellR = std::max(shellR, Vector3Distance(nodes[0].position, nodes[i].position));

        DrawSphere(nodes[0].position, shellR * 0.92f, color);
    }

    static Color randomColor()
    {
        static const Color palette[] = {
            { 80, 220,  90, 255},
            {120, 255,  60, 255},
            {200, 255,  40, 255},
            {255, 200,  50, 255},
            {255, 140,  80, 255},
            {180, 255, 120, 255},
            { 60, 200, 180, 255},
            {255, 100, 180, 255},
        };
        int n = sizeof(palette) / sizeof(palette[0]);
        return palette[GetRandomValue(0, n - 1)];
    }

    void buildSoftBody(Vector3 center, float r, float stiffness = 32.0f, float damping = 2.4f)
    {
        nodes.clear();
        springs.clear();

        nodes.push_back(Node(center, 0.06f, 0.06f / 1000.0f));

        for (int i = 0; i < SHELL_NODES; i++)
        {
            float theta = 2.0f * PI * (float)i / (float)SHELL_NODES;
            float phi   = std::acos(1.0f - 2.0f * ((float)i + 0.5f) / (float)SHELL_NODES);
            Vector3 dir = {
                std::sin(phi) * std::cos(theta),
                std::cos(phi),
                std::sin(phi) * std::sin(theta),
            };
            Vector3 pos = Vector3Add(center, Vector3Scale(dir, r));
            nodes.push_back(Node(pos, 0.015f, 0.015f / 1000.0f));
        }

        for (int i = 1; i <= SHELL_NODES; i++)
            addSpring(0, i, stiffness, damping);

        for (int i = 1; i <= SHELL_NODES; i++)
            addSpring(i, 1 + (i % SHELL_NODES), stiffness * 0.35f, damping);

        for (auto &n : nodes)
        {
            n.velocity.x = (float)GetRandomValue(-40, 40) / 400.0f;
            n.velocity.y = (float)GetRandomValue(-40, 40) / 400.0f;
            n.velocity.z = (float)GetRandomValue(-40, 40) / 400.0f;
        }
    }

    void spawnRandom(float dishRadius, float floorY, float ceilY, float margin = 1.0f)
    {
        float angle = (float)GetRandomValue(0, 360) * DEG2RAD;
        float t     = (float)GetRandomValue(0, 1000) / 1000.0f;
        float rXZ   = (dishRadius - margin) * std::sqrt(t);

        radius = 0.22f + (float)GetRandomValue(8, 28) / 100.0f;
        color  = randomColor();
        active = true;

        float yMin = floorY + radius;
        float yMax = ceilY - radius;
        if (yMax < yMin) yMax = yMin;
        float u = (float)GetRandomValue(0, 1000) / 1000.0f;

        Vector3 center = {
            rXZ * std::cos(angle),
            yMin + u * (yMax - yMin),
            rXZ * std::sin(angle),
        };

        buildSoftBody(center, radius);
    }

    void respawn(float dishRadius, float floorY, float ceilY, float margin = 1.0f)
    {
        spawnRandom(dishRadius, floorY, ceilY, margin);
    }

    void update(float dt)
    {
        if (!active) return;
        updatePhysicsImplicit(dt);
    }
};

#endif
