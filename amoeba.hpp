#pragma once
#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>

class Amoeba : public PhysicsBody
{
private:
    float baseRadius;
    float targetVolume;
    float phase;
    Vector3 heading;
    int numMembraneNodes;

public:
    // Notice: Lowered default structural stiffness (160 -> 40) to make it highly malleable/squishy
    Amoeba(Vector3 center, float radius = 2f, float stiffness =20.0f, float damping = 2.5f)
    {
        baseRadius = radius;
        phase = 0.0f;
        heading = {1.0f, 0.0f, 0.0f}; 
        numMembraneNodes = 64;

        // 1. Center Node (Nucleus) - lighter mass so it flows easily within the body
        nodes.push_back(Node(center, 0.4f)); 

        // 2. Membrane Nodes
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

        // 3. Central Springs (Highly flexible core structure)
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            addSpring(0, i, stiffness * 0.7f, damping); 
        }

        // 4. Surface Tension Springs (Loose, plastic skin allowing deep deformation)
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

    void actuate(float dt)
    {
        // Slower cycle rate (4.0 -> 2.2) to give the pseudopod elongation time to be visible
        phase += dt * 2.2f; 
        float pulse = std::sin(phase);

        // --- 1. SQUISHY VOLUME PRESERVATION ---
        float currentVolume = calculateCurrentVolume();
        float pressureStiffness = 180.0f; 
        float pressureDelta = targetVolume - currentVolume;
        float internalPressure = std::max(0.0f, pressureDelta * pressureStiffness);

        // --- 2. ANTI-GRAVITY & ENVIRONMENTAL VISCOSITY ---
        Vector3 gravityCancellation = {0.0f, 9.81f, 0.0f}; 
        const float floorY = -5.0f;

        for (size_t i = 0; i < nodes.size(); i++)
        {
            Vector3 antiGrav = Vector3Scale(gravityCancellation, nodes[i].mass);
            nodes[i].force = Vector3Add(nodes[i].force, antiGrav);

            // Slightly higher damping (0.95 -> 0.93) to make it move through a "gel" medium
            nodes[i].velocity = Vector3Scale(nodes[i].velocity, 0.93f);

            // Dynamic pressure keeps it from totally flattening, but allows pooling on the floor
            if (i > 0)
            {
                Vector3 outwardDir = Vector3Normalize(Vector3Subtract(nodes[i].position, nodes[0].position));
                Vector3 pressureForce = Vector3Scale(outwardDir, internalPressure / numMembraneNodes);
                nodes[i].force = Vector3Add(nodes[i].force, pressureForce);
            }
        }

        // --- 3. TARGETED PSEUDOPOD SHOOTING ---
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            Vector3 dirFromCenter = Vector3Normalize(Vector3Subtract(nodes[i].position, nodes[0].position));
            float alignment = Vector3DotProduct(dirFromCenter, heading);

            // High alignment cutoff (0.75) restricts actuation to a tiny point at the tip of the nose
            if (alignment > 0.75f)
            {
                if (pulse > 0.0f)
                {
                    // Explosively shoot this specific "leg" forward
                    Vector3 pseudopodThrust = Vector3Scale(heading, pulse * 28.0f * alignment * dt);
                    nodes[i].velocity = Vector3Add(nodes[i].velocity, pseudopodThrust);
                    
                    // Also expand it radially to thicken the extended leg
                    nodes[i].velocity = Vector3Add(nodes[i].velocity, Vector3Scale(dirFromCenter, pulse * 8.0f * dt));
                }
            }
            // Rear cell uroid contraction
            else if (alignment < -0.1f)
            {
                // Squeezes the back end smoothly inward towards the center mass
                Vector3 tailSqueeze = Vector3Scale(heading, 8.0f * std::abs(alignment) * dt);
                nodes[i].velocity = Vector3Add(nodes[i].velocity, tailSqueeze);
            }

            // --- LOCALIZED STICK-SLIP FRICTION ---
            if (nodes[i].position.y < floorY + 0.2f)
            {
                // ONLY anchor the tip of the newly extended pseudopod
                if (alignment > 0.6f && pulse > 0.2f)
                {
                    nodes[i].velocity.x *= 0.01f;
                    nodes[i].velocity.z *= 0.01f;
                }
                else 
                {
                    // The rest of the body has low sliding friction so it can stretch and drag smoothly
                    nodes[i].velocity.x *= 0.96f;
                    nodes[i].velocity.z *= 0.96f;
                }
            }
        }

        // Nucleus smoothly trails behind the leading foot, instead of driving the movement
        Vector3 nucleusFollow = Vector3Scale(heading, 3.5f * dt);
        nodes[0].velocity = Vector3Add(nodes[0].velocity, nucleusFollow);
    }
};