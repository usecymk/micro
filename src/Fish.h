#ifndef MICRO3D_FISH_H
#define MICRO3D_FISH_H

#include <cmath>

#include "PhysicsBody.h"

class Fish : public PhysicsBody
{
public:
    Fish(float stiffness = 8.0f, float damping = 0.6f)
    {
        int segments = 6;
        float length = 3.0f;

        nodes.reserve(segments * 4 + 3);

        for (int i = 0; i < segments; i++)
        {
            float t = (float)i / (segments - 1);
            float x = t * length;

            float radius = 0.4f * std::sin(t * 3.1415f);

            float w = radius;
            float h = radius * 0.6f;

            nodes.push_back(Node({x, 0, -w}));
            nodes.push_back(Node({x, 0, w}));
            nodes.push_back(Node({x, h, 0}));
            nodes.push_back(Node({x, -h, 0}));
        }

        int tailBase = (int)nodes.size();
        float tailX = length;

        nodes.push_back(Node({tailX + 0.5f, 0.0f, 0.0f}));
        nodes.push_back(Node({tailX + 0.8f, 0.4f, 0.0f}));
        nodes.push_back(Node({tailX + 0.8f, -0.4f, 0.0f}));

        for (int i = 0; i < segments - 1; i++)
        {
            int base = i * 4;
            int next = (i + 1) * 4;

            for (int j = 0; j < 4; j++)
                addSpring(base + j, next + j, stiffness, damping);

            addSpring(base + 0, next + 1, stiffness, damping);
            addSpring(base + 1, next + 0, stiffness, damping);
            addSpring(base + 2, next + 3, stiffness, damping);
            addSpring(base + 3, next + 2, stiffness, damping);
        }

        for (int i = 0; i < segments; i++)
        {
            int b = i * 4;

            addSpring(b + 0, b + 2, stiffness, damping);
            addSpring(b + 2, b + 1, stiffness, damping);
            addSpring(b + 1, b + 3, stiffness, damping);
            addSpring(b + 3, b + 0, stiffness, damping);

            addSpring(b + 0, b + 1, stiffness, damping);
            addSpring(b + 2, b + 3, stiffness, damping);
        }

        float tk = stiffness * 0.4f;
        int last = (segments - 1) * 4;

        addSpring(last + 0, tailBase, tk, damping);
        addSpring(last + 1, tailBase, tk, damping);
        addSpring(last + 2, tailBase, tk, damping);
        addSpring(last + 3, tailBase, tk, damping);

        addSpring(tailBase, tailBase + 1, tk, damping);
        addSpring(tailBase, tailBase + 2, tk, damping);
        addSpring(tailBase + 1, tailBase + 2, tk, damping);
    }
};

#endif
