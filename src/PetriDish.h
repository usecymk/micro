#ifndef MICRO3D_PETRI_DISH_H
#define MICRO3D_PETRI_DISH_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include "Node.h"

class PetriDish
{
public:
    float radius;
    float height;
    float floorY;

    float wallThickness = 0.18f;
    int   sides         = 40;
    int   gridLines     = 8;

    Color floorColor  = { 10,  30,  60, 200};
    Color wallColor   = { 80, 180, 220,  28};
    Color wireColor   = {120, 210, 255, 140};
    Color liquidColor = { 20,  80, 130,  18};
    Color gridColor   = { 30,  70, 120,  80};

    PetriDish(float r = 16.0f, float h = 5.0f, float floor = 0.0f)
        : radius(r), height(h), floorY(floor) {}

    float ceilY() const { return floorY + height; }


    void draw() const
    {
        Vector3 base = {0.0f, floorY, 0.0f};

        // floor disk + rim
        DrawCylinder(base, radius, radius, 0.08f, sides, floorColor);
        DrawCylinderWires(base, radius, radius, 0.08f, sides, wireColor);

        // floor grid lines clipped to the dish circle
        float step = (radius * 2.0f) / gridLines;
        float y = floorY + 0.05f;
        for (int i = 0; i <= gridLines; i++)
        {
            float t = -radius + i * step;
            float halfLen = std::sqrt(std::max(0.0f, radius * radius - t * t));
            DrawLine3D({t, y, -halfLen}, {t, y,  halfLen}, gridColor);
            DrawLine3D({-halfLen, y, t}, { halfLen, y, t}, gridColor);
        }
    }

    void drawShell() const
    {
        float top = ceilY();

        rlDisableDepthMask();

        rlBegin(RL_QUADS);
        rlColor4ub(wallColor.r, wallColor.g, wallColor.b, wallColor.a);
        for (int i = 0; i < sides; i++)
        {
            float a0 = (float)i       / sides * 2.0f * PI;
            float a1 = (float)(i + 1) / sides * 2.0f * PI;
            float x0 = std::cos(a0) * radius, z0 = std::sin(a0) * radius;
            float x1 = std::cos(a1) * radius, z1 = std::sin(a1) * radius;

            // outer face
            rlVertex3f(x0, floorY, z0);
            rlVertex3f(x1, floorY, z1);
            rlVertex3f(x1, top,    z1);
            rlVertex3f(x0, top,    z0);
            // inner face (reverse winding so it's visible from inside too)
            rlVertex3f(x0, top,    z0);
            rlVertex3f(x1, top,    z1);
            rlVertex3f(x1, floorY, z1);
            rlVertex3f(x0, floorY, z0);
        }
        rlEnd();

        Vector3 base = {0.0f, floorY, 0.0f};
        DrawCylinder(base,
                     radius - wallThickness,
                     radius - wallThickness,
                     height, sides, liquidColor);

        rlEnableDepthMask();

 
        DrawCylinderWires(base, radius, radius, height, sides, wireColor);
    }

    // restitution: 0 = fully inelastic, 1 = perfect elastic bounce.
    // `bodyRadius` keeps the contact point slightly above the floor so
    // capsule-rendered bodies don't visibly clip through it.
    void applyBoundary(Node &n, float restitution = 0.4f, float bodyRadius = 0.18f) const
    {
        // floor
        if (n.position.y < floorY + bodyRadius)
        {
            n.position.y = floorY + bodyRadius;
            if (n.velocity.y < 0.0f)
                n.velocity.y = -n.velocity.y * restitution;
        }

        // ceiling (liquid surface)
        float top = ceilY();
        if (n.position.y > top)
        {
            n.position.y = top;
            if (n.velocity.y > 0.0f)
                n.velocity.y = -n.velocity.y * restitution;
        }

        // cylindrical side wall: reflect the radial component of velocity
        float dx = n.position.x;
        float dz = n.position.z;
        float r  = std::sqrt(dx * dx + dz * dz);
        if (r > radius)
        {
            float scale = radius / r;
            n.position.x *= scale;
            n.position.z *= scale;

            float nx = dx / r;
            float nz = dz / r;
            float vn = n.velocity.x * nx + n.velocity.z * nz;
            if (vn > 0.0f)
            {
                n.velocity.x -= (1.0f + restitution) * vn * nx;
                n.velocity.z -= (1.0f + restitution) * vn * nz;
            }
        }
    }

    void applyBoundary(std::vector<Node> &nodes, float restitution = 0.4f, float bodyRadius = 0.18f) const
    {
        for (auto &n : nodes)
            applyBoundary(n, restitution, bodyRadius);
    }
};

#endif
