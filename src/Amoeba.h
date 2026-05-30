#ifndef MICRO3D_AMOEBA_H
#define MICRO3D_AMOEBA_H

#include <algorithm>
#include <cmath>

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"

class Amoeba : public PhysicsBody
{
public:
    bool  isHuntingBait = true;
    float searchRadius  = 25.0f;

    Amoeba(Vector3 center,
           float radius    = 2.0f,
           float stiffness = 20.0f,
           float damping   = 2.5f)
        : baseRadius(radius),
          phase(0.0f),
          heading({1.0f, 0.0f, 0.0f}),
          numMembraneNodes(64),
          floorY(-1.0e9f)
    {
        nodes.push_back(Node(center, 0.4f));

        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t      = (float)i / (float)(numMembraneNodes - 1);
            float y      = 1.0f - (t * 2.0f);
            float ringR  = std::sqrt(std::max(0.0f, 1.0f - y * y));
            float theta  = PI * (1.0f + std::sqrt(5.0f)) * (float)i;

            float x = std::cos(theta) * ringR;
            float z = std::sin(theta) * ringR;

            Vector3 offset = Vector3Scale({x, y, z}, radius);
            nodes.push_back(Node(Vector3Add(center, offset), 0.2f));
        }

        for (int i = 1; i <= numMembraneNodes; i++)
            addSpring(0, i, stiffness * 0.7f, damping);

        float expected  = std::sqrt(4.0f * PI * radius * radius / (float)numMembraneNodes);
        float threshold = expected * 1.5f;
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            for (int j = i + 1; j <= numMembraneNodes; j++)
            {
                float d = Vector3Distance(nodes[i].position, nodes[j].position);
                if (d < threshold)
                    addSpring(i, j, stiffness * 0.8f, damping);
            }
        }
    }

    void setFloorY(float y) { floorY = y; }

    Vector3 getCenterPosition() const
    {
        return nodes.empty() ? Vector3Zero() : nodes[0].position;
    }

    void actuate(float dt, Vector3 targetPos)
    {
        if (nodes.empty())
            return;

        if (isHuntingBait)
        {
            Vector3 toPrey = Vector3Subtract(targetPos, nodes[0].position);
            toPrey.y = 0.0f;
            float distToPrey = Vector3Length(toPrey);
            if (distToPrey < searchRadius && distToPrey > 0.1f)
            {
                Vector3 desired = Vector3Normalize(toPrey);
                heading = Vector3Lerp(heading, desired, dt * 2.0f);
                heading = Vector3Normalize(heading);
            }
        }

        phase += dt * 2.2f;
        float pulse = std::sin(phase);

        for (auto &n : nodes)
            n.velocity = Vector3Scale(n.velocity, 0.93f);

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            Vector3 dirFromCenter = Vector3Normalize(Vector3Subtract(nodes[i].position, nodes[0].position));
            float   alignment     = Vector3DotProduct(dirFromCenter, heading);

            if (alignment > 0.75f && pulse > 0.0f)
            {
                Vector3 thrust = Vector3Scale(heading,       pulse * 28.0f * alignment * dt);
                Vector3 puff   = Vector3Scale(dirFromCenter, pulse *  8.0f             * dt);
                nodes[i].velocity = Vector3Add(nodes[i].velocity, thrust);
                nodes[i].velocity = Vector3Add(nodes[i].velocity, puff);
            }
            else if (alignment < -0.1f)
            {
                Vector3 squeeze = Vector3Scale(heading, 8.0f * std::abs(alignment) * dt);
                nodes[i].velocity = Vector3Add(nodes[i].velocity, squeeze);
            }

            if (nodes[i].position.y < floorY + 0.2f)
            {
                float lateral = (alignment > 0.6f && pulse > 0.2f) ? 0.01f : 0.96f;
                nodes[i].velocity.x *= lateral;
                nodes[i].velocity.z *= lateral;
            }
        }

        nodes[0].velocity = Vector3Add(nodes[0].velocity, Vector3Scale(heading, 3.5f * dt));
    }

    void draw()
    {
        for (size_t i = 1; i < nodes.size(); i++)
            DrawSphere(nodes[i].position, 0.1f, PURPLE);

        DrawSphere(nodes[0].position, 0.25f, RED);

        for (auto &s : springs)
            DrawLine3D(s.nodeA->position, s.nodeB->position, Fade(WHITE, 0.3f));
    }

private:
    float   baseRadius;
    float   phase;
    Vector3 heading;
    int     numMembraneNodes;
    float   floorY;
};

#endif
