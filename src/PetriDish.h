#ifndef MICRO3D_PETRI_DISH_H
#define MICRO3D_PETRI_DISH_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>

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

    Color floorColor  = { 10,  30,  60, 255};
    Color wallColor   = { 80, 180, 220,  60};
    Color wireColor   = {120, 210, 255, 180};
    Color liquidColor = { 20,  80, 130,  40};
    Color gridColor   = { 30,  70, 120,  80};

    PetriDish(float r = 8.0f, float h = 5.0f, float floor = 0.0f)
        : radius(r), height(h), floorY(floor) {}

    float ceilY() const { return floorY + height; }

    void draw() const
    {
        Vector3 base = {0.0f, floorY, 0.0f};

        // floor disk + rim
        DrawCylinder(base, radius, radius, 0.08f, sides, floorColor);
        DrawCylinderWires(base, radius, radius, 0.08f, sides, wireColor);

        // semi-transparent outer wall
        DrawCylinder(base, radius, radius, height, sides, wallColor);
        DrawCylinderWires(base, radius, radius, height, sides, wireColor);

        // faint liquid fill (inner cylinder)
        DrawCylinder(base,
                     radius - wallThickness,
                     radius - wallThickness,
                     height, sides, liquidColor);

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

    void applyBoundary(std::vector<Node> &nodes, float restitution = 0.4f) const
    {
        for (auto &n : nodes)
            applyBoundary(n, restitution);
    }
};

#endif
