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
    struct HeatSource
    {
        Vector3 position;
        float peakTemperature;
        float sigma;
        float boundary;
        float lampRadius;
    };

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

    float   baseTemperature = 22.0f;
    std::vector<HeatSource> heatSources;

    PetriDish(float r = 16.0f, float h = 5.0f, float floor = 0.0f)
        : radius(r), height(h), floorY(floor)
    {
        heatSources.push_back({
            {radius + 1.2f, floorY + height * 0.62f, 0.0f}, // outside heat lamp, on +X side
            28.0f,
            10.8f,
            0.8f,
            0.26f
        });
    }

    float ceilY() const { return floorY + height; }

    float temperatureAt(Vector3 pos) const
    {
        float temp = baseTemperature;
        for (const auto &source : heatSources)
        {
            float sigma = std::max(source.sigma, 1e-4f);
            Vector3 toPoint = Vector3Subtract(pos, source.position);
            float distance = Vector3Length(toPoint);
            float warmed = std::exp(-(distance * distance) / (2.0f * sigma * sigma));
            temp += source.peakTemperature * warmed;
        }
        return temp;
    }

    Vector3 temperatureGradientAt(Vector3 pos) const
    {
        Vector3 gradient = Vector3Zero();
        for (const auto &source : heatSources)
        {
            float sigma = std::max(source.sigma, 1e-4f);
            float sigma2 = sigma * sigma;
            Vector3 toPoint = Vector3Subtract(pos, source.position);
            float distance2 = Vector3DotProduct(toPoint, toPoint);
            float warmed = std::exp(-distance2 / (2.0f * sigma2));
            Vector3 sourceGradient = Vector3Scale(toPoint, -source.peakTemperature * warmed / sigma2);
            gradient = Vector3Add(gradient, sourceGradient);
        }
        return gradient;
    }

    void draw(float targetTemperature = 40.0f) const
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

        // Dotted 3D isothermal shell where this heat source reaches targetTemperature.
        for (const auto &source : heatSources)
        {
            float ratio = (targetTemperature - baseTemperature) / std::max(source.peakTemperature, 1e-4f);
            if (ratio <= 0.0f || ratio >= 1.0f)
                continue;

            float shellRadius = source.sigma * std::sqrt(-2.0f * std::log(ratio));
            for (int lat = -15; lat <= 15; lat++)
            {
                float theta = (float)lat / 15.0f * (PI * 0.5f);
                float ringR = std::cos(theta);
                float y = std::sin(theta);
                for (int lon = 0; lon < 84; lon++)
                {
                    float phi = ((float)lon / 84.0f) * 2.0f * PI;
                    Vector3 offset = {
                        std::cos(phi) * ringR,
                        y,
                        std::sin(phi) * ringR
                    };
                    Vector3 p = Vector3Add(source.position, Vector3Scale(offset, shellRadius));
                    float radial = std::sqrt(p.x * p.x + p.z * p.z);
                    if (radial > radius || p.y < floorY || p.y > ceilY())
                        continue;

                    DrawSphere(p, 0.045f, {255, 150, 55, 125});
                }
            }
        }

        // outside heat lamp markers
        for (const auto &source : heatSources)
        {
            Vector3 lamp = source.position;
            DrawSphere(lamp, source.lampRadius * 0.70f, {255, 180, 60, 240});
            DrawSphereWires(lamp, source.lampRadius, 12, 8, {255, 120, 30, 200});
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
