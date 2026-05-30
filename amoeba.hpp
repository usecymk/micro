#pragma once
#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>
#include "bait.hpp"
#include "BehaviorStateMachine.h"

class Amoeba : public PhysicsBody
{
private:
    float baseRadius;
    float targetVolume;
    float phase;
    Vector3 heading;
    int numMembraneNodes;
    BehaviorStateMachine myStateMachine; 

public:
    // --- BEHAVIOR TOGGLES ---
    bool isHuntingBait = true;
    float searchRadius = 25.0f;

    Amoeba(Vector3 center, float radius = 2.0f, float stiffness = 20.0f, float damping = 2.5f)
    {
        baseRadius = radius;
        phase = 0.0f;
        heading = {1.0f, 0.0f, 0.0f}; 
        numMembraneNodes = 64;

        nodes.push_back(Node(center, 0.4f)); 

        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t = (float)i / (numMembraneNodes - 1); 
            float y = 1.0f - (t * 2.0f); 
            float radius_at_y = std::sqrt(1.0f - y * y);
            float theta = PI * (1.0f + std::sqrt(5.0f)) * i; 
            
            float x = std::cos(theta) * radius_at_y;
            float z = std::sin(theta) * radius_at_y;

            Vector3 offset = Vector3Scale({x, y, z}, radius);
            nodes.push_back(Node(Vector3Add(center, offset), 0.2f)); 
        }

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            addSpring(0, i, stiffness * 0.7f, damping); 
        }

        float expected_dist = std::sqrt(4.0f * PI * radius * radius / numMembraneNodes);
        float threshold = expected_dist * 1.5f; 

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            for (int j = i + 1; j <= numMembraneNodes; j++)
            {
                float d = Vector3Distance(nodes[i].position, nodes[j].position);
                if (d < threshold)
                {
                    addSpring(i, j, stiffness * 0.8f, damping);
                }
            }
        }

        targetVolume = (4.0f / 3.0f) * PI * std::pow(radius, 3);
    }

    float calculateCurrentVolume()
    {
        float currentRadiusSum = 0.0f;
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            currentRadiusSum += Vector3Distance(nodes[i].position, nodes[0].position);
        }
        float avgRadius = currentRadiusSum / numMembraneNodes;
        return (4.0f / 3.0f) * PI * std::pow(avgRadius, 3);
    }

    void actuate(float dt, Vector3 targetPos)
    {
        // --- BAIT TRACKING LOGIC ---
        if (isHuntingBait)
        {
            Vector3 toPrey = Vector3Subtract(targetPos, nodes[0].position);
            toPrey.y = 0.0f; // Keep steering horizontal

            float distToPrey = Vector3Length(toPrey);

            if (distToPrey < searchRadius && distToPrey > 0.1f)
            {
                Vector3 desiredHeading = Vector3Normalize(toPrey);
                heading = Vector3Lerp(heading, desiredHeading, dt * 2.0f);
                heading = Vector3Normalize(heading);
            }
        }

        phase += dt * 2.2f; 
        float pulse = std::sin(phase);

        float currentVolume = calculateCurrentVolume();
        float pressureStiffness = 180.0f; 
        float pressureDelta = targetVolume - currentVolume;
        float internalPressure = std::max(0.0f, pressureDelta * pressureStiffness);

        Vector3 gravityCancellation = {0.0f, 9.81f, 0.0f}; 
        const float floorY = -5.0f;

        for (size_t i = 0; i < nodes.size(); i++)
        {
            Vector3 antiGrav = Vector3Scale(gravityCancellation, nodes[i].mass);
            nodes[i].force = Vector3Add(nodes[i].force, antiGrav);

            nodes[i].velocity = Vector3Scale(nodes[i].velocity, 0.93f);

            if (i > 0)
            {
                Vector3 outwardDir = Vector3Normalize(Vector3Subtract(nodes[i].position, nodes[0].position));
                Vector3 pressureForce = Vector3Scale(outwardDir, internalPressure / numMembraneNodes);
                nodes[i].force = Vector3Add(nodes[i].force, pressureForce);
            }
        }

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            Vector3 dirFromCenter = Vector3Normalize(Vector3Subtract(nodes[i].position, nodes[0].position));
            float alignment = Vector3DotProduct(dirFromCenter, heading);

            if (alignment > 0.75f)
            {
                if (pulse > 0.0f)
                {
                    Vector3 pseudopodThrust = Vector3Scale(heading, pulse * 28.0f * alignment * dt);
                    nodes[i].velocity = Vector3Add(nodes[i].velocity, pseudopodThrust);
                    
                    nodes[i].velocity = Vector3Add(nodes[i].velocity, Vector3Scale(dirFromCenter, pulse * 8.0f * dt));
                }
            }
            else if (alignment < -0.1f)
            {
                Vector3 tailSqueeze = Vector3Scale(heading, 8.0f * std::abs(alignment) * dt);
                nodes[i].velocity = Vector3Add(nodes[i].velocity, tailSqueeze);
            }

            if (nodes[i].position.y < floorY + 0.2f)
            {
                if (alignment > 0.6f && pulse > 0.2f)
                {
                    nodes[i].velocity.x *= 0.01f;
                    nodes[i].velocity.z *= 0.01f;
                }
                else 
                {
                    nodes[i].velocity.x *= 0.96f;
                    nodes[i].velocity.z *= 0.96f;
                }
            }
        }

        Vector3 nucleusFollow = Vector3Scale(heading, 3.5f * dt);
        nodes[0].velocity = Vector3Add(nodes[0].velocity, nucleusFollow);
    }
};