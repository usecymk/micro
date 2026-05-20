#pragma once
#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>

class Amoeba : public PhysicsBody
{
private:
    float baseRadius;
    std::vector<float> baseRestLengths;
    float phase;
    Vector3 heading;

public:
    Amoeba(Vector3 center, float radius = 1.5f, float stiffness = 30.0f, float damping = 3.0f)
    {
        baseRadius = radius;
        phase = 0.0f;
        heading = {1.0f, 0.0f, 0.0f}; // Default movement along +X axis

        // 1. Center Node (The bulk mass / nucleus)
        nodes.push_back(Node(center, 3.0f)); 

        // 2. Membrane Nodes (Fibonacci Sphere for even distribution)
        int numMembraneNodes = 64; 
        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t = (float)i / (numMembraneNodes - 1); 
            float y = 1.0f - (t * 2.0f); 
            float radius_at_y = std::sqrt(1.0f - y * y);
            float theta = PI * (1.0f + std::sqrt(5.0f)) * i; 
            
            float x = std::cos(theta) * radius_at_y;
            float z = std::sin(theta) * radius_at_y;

            Vector3 offset = Vector3Scale({x, y, z}, radius);
            nodes.push_back(Node(Vector3Add(center, offset), 0.3f)); 
        } // <-- Fixed: This brace was missing/misplaced!

        // 3. Cytoplasm Springs (Internal Pressure)
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            addSpring(0, i, stiffness * 0.4f, damping); 
        }

        // 4. Membrane Springs (Surface Tension)
        float expected_dist = std::sqrt(4.0f * PI * radius * radius / numMembraneNodes);
        float threshold = expected_dist * 1.4f; 

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            for (int j = i + 1; j <= numMembraneNodes; j++)
            {
                float d = Vector3Distance(nodes[i].position, nodes[j].position);
                if (d < threshold)
                {
                    addSpring(i, j, stiffness, damping);
                }
            }
        }

        // Store original rest lengths for the motor
        for (auto &sp : springs)
        {
            baseRestLengths.push_back(sp.rest_length);
        }
    }

    void actuate(float dt)
    {
        phase += dt * 3.5f; 
        float pulse = std::sin(phase);

        for (size_t i = 0; i < springs.size(); i++)
        {
            if (springs[i].nodeA == &nodes[0] || springs[i].nodeB == &nodes[0])
            {
                Node *surfaceNode = (springs[i].nodeA == &nodes[0]) ? springs[i].nodeB : springs[i].nodeA;

                Vector3 dirToNode = Vector3Normalize(Vector3Subtract(surfaceNode->position, nodes[0].position));
                float alignment = Vector3DotProduct(dirToNode, heading);

                float new_length = baseRestLengths[i];

                if (alignment > 0.3f) 
                {
                    float extension = (pulse > 0.0f) ? pulse : 0.0f;
                    new_length += baseRadius * 0.8f * extension * alignment;
                }
                else if (alignment < -0.2f) 
                {
                    new_length -= baseRadius * 0.4f * std::abs(alignment);
                }

                springs[i].rest_length = new_length;

                const float floorY = -5.0f;
                if (surfaceNode->position.y < floorY + 0.15f)
                {
                    if (alignment > 0.0f && pulse > 0.0f)
                    {
                        surfaceNode->velocity.x *= 0.05f;
                        surfaceNode->velocity.z *= 0.05f;
                    }
                    else if (alignment < 0.0f)
                    {
                        surfaceNode->velocity.x *= 0.85f;
                        surfaceNode->velocity.z *= 0.85f;
                    }
                }
            }
        }
    }
};