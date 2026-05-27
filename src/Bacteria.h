#ifndef MICRO3D_BACTERIA_H
#define MICRO3D_BACTERIA_H

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"

class Bacteria : public PhysicsBody
{
public:
    enum : int
    {
        BODY_NODES = 4,
        FLAG_NODES = 14,
        TOTAL_NODES = BODY_NODES + FLAG_NODES,
    };

    Bacteria(float stiffness = 60.0f, float damping = 2.0f)
    {
        float segLen = 0.18f; // spacing between flagellum nodes

        // body nodes (stiff pill along +Z)
        nodes.push_back(Node({0.0f, 2.0f, -0.4f }));  // 0  front of body
        nodes.push_back(Node({0.0f, 2.0f, -0.13f}));  // 1
        nodes.push_back(Node({0.0f, 2.0f,  0.13f}));  // 2
        nodes.push_back(Node({0.0f, 2.0f,  0.4f }));  // 3  back of body

        // flagellum nodes trailing behind the body
        for (int i = 0; i < FLAG_NODES; i++)
        {
            float z = 0.4f + (i + 1) * segLen;
            nodes.push_back(Node({0.0f, 2.0f, z}));
        }

        float k = stiffness;
        float d = damping;

        // rigid body springs
        addSpring(0, 1, k, d);
        addSpring(1, 2, k, d);
        addSpring(2, 3, k, d);
        addSpring(0, 2, k * 0.5f, d);
        addSpring(1, 3, k * 0.5f, d);
        addSpring(0, 3, k * 0.3f, d);

        // soft "muscle" springs along the flagellum
        float fk = k * 0.02f;
        float fd = d * 0.05f;
        for (int i = 0; i < FLAG_NODES; i++)
            addSpring(BODY_NODES - 1 + i, BODY_NODES + i, fk, fd);
    }

    void draw()
    {
        auto &ns = getNodes();

        // green pill body
        float radius = 0.18f;
        Color bodyCol = {80, 220, 120, 255};
        for (int i = 0; i < BODY_NODES - 1; i++)
            DrawCapsule(ns[i].position, ns[i + 1].position, radius, 6, 4, bodyCol);
        DrawSphere(ns[0].position, radius, bodyCol);
        DrawSphere(ns[BODY_NODES - 1].position, radius, bodyCol);

        // blue flagellum, tapering to a thin tip
        for (int i = BODY_NODES - 1; i < TOTAL_NODES - 1; i++)
        {
            float t = (float)(i - (BODY_NODES - 1)) / (float)FLAG_NODES;
            float thickness = 0.055f * (1.0f - t * 0.75f);
            Color col = {100, 210, 255, 255};
            DrawCapsule(ns[i].position, ns[i + 1].position, thickness, 4, 2, col);
        }
    }
};

#endif
