#ifndef MICRO3D_AMOEBA_H
#define MICRO3D_AMOEBA_H

#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <raylib.h>
#include <raymath.h>
#include "Cocci.h"
#include "BoidBehavior.h" 
#include "PredatorBehaviorStateMachine.h"

class Amoeba : public PhysicsBody
{
private:
    float baseRadius;
    float targetVolume;
    float phase;
    Vector3 heading;
    int numMembraneNodes;

    PredatorBehaviorStateMachine myStateMachine; 

    PredatorDecision lastPredatorDecision;

    float floorY = -5.0f; 

    static unsigned char colorChannel(float value)
    {
        return (unsigned char)Clamp(value, 0.0f, 255.0f);
    }

    static Color lerpColor(Color from, Color to, float amount)
    {
        amount = Clamp(amount, 0.0f, 1.0f);
        return {
            colorChannel((float)from.r + ((float)to.r - (float)from.r) * amount),
            colorChannel((float)from.g + ((float)to.g - (float)from.g) * amount),
            colorChannel((float)from.b + ((float)to.b - (float)from.b) * amount),
            colorChannel((float)from.a + ((float)to.a - (float)from.a) * amount)
        };
    }

    Color getIntentBaseColor() const
    {
        switch (lastPredatorDecision.intent)
        {
            case PredatorIntent::AVOID:
                // Cyan-blue: backing away from the dish boundary.
                return {75, 210, 255, 58};
            case PredatorIntent::EAT:
                // Amber: actively hunting or eating prey.
                return {245, 190, 55, 62};
            case PredatorIntent::SEEK_TEMP:
                // Violet: moving toward a more comfortable temperature.
                return {185, 115, 255, 52};
            case PredatorIntent::WANDER:
            default:
                // Aqua-green: calm wandering/default state.
                return {21, 228, 179, 38};
        }
    }

    Color getMembraneColor() const
    {
        float hunger = Clamp(myStateMachine.getHunger(), 0.0f, 1.0f);
        float urgency = Clamp(lastPredatorDecision.urgency, 0.0f, 1.0f);
        float stress = std::max(hunger, urgency);

        Color color = getIntentBaseColor();
        color = lerpColor(color, {255, 82, 38, 92}, hunger * 0.45f);

        switch (lastPredatorDecision.intent)
        {
            case PredatorIntent::EAT:
                color = lerpColor(color, {255, 42, 24, 122}, urgency * 0.65f);
                break;
            case PredatorIntent::AVOID:
                color = lerpColor(color, {110, 235, 255, 112}, 0.35f + urgency * 0.35f);
                break;
            case PredatorIntent::SEEK_TEMP:
                color = lerpColor(color, {255, 145, 55, 92}, urgency * 0.35f);
                break;
            case PredatorIntent::WANDER:
            default:
                color = lerpColor(color, {120, 255, 205, 70}, stress * 0.20f);
                break;
        }

        if (lastPredatorDecision.captured)
            color = lerpColor(color, {255, 245, 180, 150}, 0.55f);

        color.a = colorChannel(35.0f + stress * 75.0f + (lastPredatorDecision.captured ? 40.0f : 0.0f));
        return color;
    }

public:
    // --- BEHAVIOR TOGGLES ---
    float searchRadius = 11.6667f;

    Amoeba(Vector3 center, float radius = 0.6667f, float stiffness = 12.0f, float damping = 1.8f)
    {
        baseRadius = radius;
        phase = 0.0f;
        heading = {1.0f, 0.0f, 0.0f}; 
        numMembraneNodes = 64;
        
        searchRadius = 11.6667f; 

        nodes.push_back(Node(center, 0.15f)); 

        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t = (float)i / (numMembraneNodes - 1); 
            float y = 1.0f - (t * 2.0f); 
            float radius_at_y = std::sqrt(1.0f - y * y);
            float theta = PI * (1.0f + std::sqrt(5.0f)) * i; 
            
            float x = std::cos(theta) * radius_at_y;
            float z = std::sin(theta) * radius_at_y;

            Vector3 offset = Vector3Scale({x, y, z}, radius);
            nodes.push_back(Node(Vector3Add(center, offset), 0.07f)); 
        }

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            addSpring(0, i, stiffness * 0.6f, damping); 
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
                    addSpring(i, j, stiffness * 0.7f, damping);
                }
            }
        }

        targetVolume = (4.0f / 3.0f) * PI * std::pow(radius, 3);
    }

    void setFloorY(float y) 
    { 
        floorY = y; 
    }

    Vector3 getCenterPosition() const 
    { 
        if (nodes.empty()) return Vector3Zero();
        return nodes[0].position;
    }

    const PredatorDecision &getLastPredatorDecision() const
    {
        return lastPredatorDecision;
    }

    void onConsumedPrey()
    {
        myStateMachine.onConsumePrey();
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

    void actuate(float dt,
                 const std::vector<BoidState>& flockStates,
                 float arenaRadius = 8.0f,
                 const CocciCluster* cocci = nullptr,
                 float currentTemp = 40.0f,
                 Vector3 tempGradient = {0.0f, 0.0f, 0.0f})
    {
        Vector3 myPos = nodes[0].position;

        std::vector<PreyCandidate> preyCandidates;
        if (cocci)
        {
            preyCandidates.emplace_back(
                -1,
                cocci->getCenterPosition(),
                1.0f,
                0.8f,
                0.3f
            );
        }

        for (int i = 0; i < (int)flockStates.size(); i++)
        {
            preyCandidates.emplace_back(
                i,
                flockStates[i].position,
                0.12f,
                0.2f,
                0.8f
            );
        }

        lastPredatorDecision = myStateMachine.updatePredator(
            dt, myPos, preyCandidates, arenaRadius, currentTemp, tempGradient);

        float speedMultiplier = 1.0f;
        float stretchFactor = 1.0f;
        float pulseSpeed = 2.2f;
        Vector3 targetHeading = heading;

        if (lastPredatorDecision.intent == PredatorIntent::AVOID)
        {
            speedMultiplier = 1.6f;
            stretchFactor = 1.3f;
            pulseSpeed = 2.5f;

            Vector3 toTarget = Vector3Subtract(lastPredatorDecision.target, myPos);
            toTarget.y = 0.0f;
            if (Vector3Length(toTarget) > 0.033f)
                targetHeading = Vector3Normalize(toTarget);
        }
        else if (lastPredatorDecision.intent == PredatorIntent::EAT)
        {
            speedMultiplier = 0.45f + 0.35f * lastPredatorDecision.urgency;
            stretchFactor = 0.65f + 0.25f * lastPredatorDecision.urgency;
            pulseSpeed = 1.1f + 0.5f * lastPredatorDecision.urgency;

            Vector3 toPrey = Vector3Subtract(lastPredatorDecision.target, myPos);
            toPrey.y *= 0.65f;
            if (Vector3Length(toPrey) > 0.033f)
            {
                targetHeading = Vector3Normalize(toPrey);
            }
        }
        else if (lastPredatorDecision.intent == PredatorIntent::SEEK_TEMP)
        {
            speedMultiplier = 0.35f;
            stretchFactor = 0.55f;
            pulseSpeed = 0.95f;

            Vector3 toTarget = Vector3Subtract(lastPredatorDecision.target, myPos);
            toTarget.y *= 0.5f;
            if (Vector3Length(toTarget) > 0.033f)
                targetHeading = Vector3Normalize(toTarget);
        }
        else 
        {
            speedMultiplier = 0.25f;
            stretchFactor = 0.45f;
            pulseSpeed = 0.65f;

            Vector3 toWander = Vector3Subtract(lastPredatorDecision.target, myPos);
            toWander.y = 0.0f;
            if (Vector3Length(toWander) > 0.033f)
                targetHeading = Vector3Normalize(toWander);
        }

        const float amoebaMotionScale = 0.35f;
        speedMultiplier *= amoebaMotionScale;
        stretchFactor *= 0.45f;

        heading = Vector3Lerp(heading, targetHeading, dt * 3.5f);
        heading = Vector3Normalize(heading);

        phase += dt * pulseSpeed; 
        float pulse = std::sin(phase);

        float currentVolume = calculateCurrentVolume();
        float pressureStiffness = 320.0f; 
        float pressureDelta = targetVolume - currentVolume;
        float internalPressure = std::max(0.0f, pressureDelta * pressureStiffness);

        Vector3 gravityCancellation = {0.0f, 9.81f, 0.0f}; 

        for (size_t i = 0; i < nodes.size(); i++)
        {
            Vector3 antiGrav = Vector3Scale(gravityCancellation, nodes[i].mass);
            nodes[i].force = Vector3Add(nodes[i].force, antiGrav);
            nodes[i].velocity = Vector3Scale(nodes[i].velocity, 0.95f);

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

            if (alignment > 0.45f)
            {
                if (pulse > 0.0f)
                {
                    float thrustMagnitude = pulse * 36.6f * speedMultiplier * std::pow(alignment, 2) * dt;
                    Vector3 pseudopodThrust = Vector3Scale(heading, thrustMagnitude);
                    nodes[i].velocity = Vector3Add(nodes[i].velocity, pseudopodThrust);
                    
                    float extensionMagnitude = pulse * 11.6f * stretchFactor * alignment * dt;
                    Vector3 pseudopodStretch = Vector3Scale(dirFromCenter, extensionMagnitude);
                    nodes[i].force = Vector3Add(nodes[i].force, pseudopodStretch);
                }
            }
            else if (alignment < -0.1f)
            {
                float squeezeMagnitude = 15.0f * speedMultiplier * stretchFactor * std::abs(alignment) * dt;
                Vector3 tailSqueeze = Vector3Scale(heading, squeezeMagnitude);
                nodes[i].velocity = Vector3Add(nodes[i].velocity, tailSqueeze);
            }

            if (nodes[i].position.y < floorY + 0.0667f)
            {
                if (alignment > 0.5f && pulse > 0.2f)
                {
                    nodes[i].velocity.x *= 0.01f;
                    nodes[i].velocity.z *= 0.01f;
                }
                else 
                {
                    nodes[i].velocity.x *= 0.92f;
                    nodes[i].velocity.z *= 0.92f;
                }
            }
        }

        Vector3 nucleusFollow = Vector3Scale(heading, 7.333f * speedMultiplier * dt);
        nodes[0].velocity = Vector3Add(nodes[0].velocity, nucleusFollow);
    }

    void draw(bool debugOverlay = false)
    {
        if (nodes.empty()) return;

        Color faceColor = getMembraneColor();

        int numNodes = (int)nodes.size();
        
        bool connected[128][128];
        int safeLimit = std::min(numNodes, 128);
        
        for (int i = 0; i < safeLimit; ++i)
        {
            for (int j = 0; j < safeLimit; ++j)
            {
                connected[i][j] = false;
            }
        }

        for (const auto& s : springs)
        {
            int u = s.nodeA - &nodes[0];
            int v = s.nodeB - &nodes[0];
            
            if (u >= 0 && u < safeLimit && v >= 0 && v < safeLimit)
            {
                connected[u][v] = true;
                connected[v][u] = true;
            }
        }

        for (int u = 1; u < safeLimit; u++)
        {
            for (int v = u + 1; v < safeLimit; v++)
            {
                if (!connected[u][v]) continue;

                for (int w = v + 1; w < safeLimit; w++)
                {
                    if (connected[u][w] && connected[v][w])
                    {
                        DrawTriangle3D(nodes[u].position, nodes[v].position, nodes[w].position, faceColor);
                        DrawTriangle3D(nodes[u].position, nodes[w].position, nodes[v].position, faceColor);
                    }
                }
            }
        }

        if (debugOverlay)
        {
            Vector3 com = nodes[0].position;
            DrawLine3D(com, Vector3Add(com, Vector3Scale(heading, 2.5f)), LIME);
            DrawSphere(Vector3Add(com, Vector3Scale(heading, 2.5f)), 0.12f, LIME);

            Vector3 worldUp = {0.0f, 1.0f, 0.0f};
            Vector3 localRight = Vector3Normalize(Vector3CrossProduct(heading, worldUp));
            DrawLine3D(com, Vector3Add(com, Vector3Scale(localRight, 1.5f)), PURPLE);
        }
    }
};

#endif
