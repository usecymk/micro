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
    float internalHunger = 0.0f;
    float internalFear = 0.0f;
    float internalTempStress = 0.0f;

    Vector3 wanderTargetHeading;
    float wanderTimer = 0.0f;

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
        if (myStateMachine.isHungry()) {
            // Hungry: amber/orange
            return {255, 170, 35, 72};
        } else if (myStateMachine.isSeekingTemperature()) {
            // Temperature stress: violet
            return {145, 105, 255, 64};
        } else {
            // Idle/wandering: calm aqua-green
            return {35, 215, 170, 48};
        }
    }

    Color getMembraneColor() const
    {
        Color color = getIntentBaseColor();
        float hunger01 = internalHunger / 100.0f;
        float tempStress01 = internalTempStress / 100.0f;

        color = lerpColor(color, {255, 120, 45, 92}, hunger01 * 0.30f);

        if (myStateMachine.isHungry()) {
            color = lerpColor(color, {255, 55, 24, 132}, hunger01 * 0.70f);
        } else if (myStateMachine.isSeekingTemperature()) {
            color = lerpColor(color, {255, 135, 55, 118}, tempStress01 * 0.55f);
        }

        return color;
    }

public:
    // --- BEHAVIOR TOGGLES ---
    float searchRadius = 11.6667f;
    float preySizeCostWeight = 0.35f;
    float preyEscapeCostWeight = 0.45f;

    std::vector<Vector3> pendingForces;

    // A custom generator that PhysicsBody will call at the right time
    class AmoebaForceInjector : public ForceGenerator
    {
        Amoeba* amoeba;
    public:
        AmoebaForceInjector(Amoeba* a) : amoeba(a) {}
        void apply(PhysicsBody& body, float dt) override
        {
            auto& bodyNodes = body.getNodes();
            for (size_t i = 0; i < bodyNodes.size(); i++)
            {
                // Inject the forces that actuate() calculated
                bodyNodes[i].force = Vector3Add(bodyNodes[i].force, amoeba->pendingForces[i]);
            }
        }
    };

    Amoeba(Vector3 center, float radius = 0.6667f, float stiffness = 12.0f, float damping = 1.8f)
    {
        baseRadius = radius;
        phase = 0.0f;
        heading = {1.0f, 0.0f, 0.0f}; 
        wanderTargetHeading = heading;
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

        pendingForces.resize(nodes.size(), Vector3Zero());
        addForceGenerator(std::make_unique<AmoebaForceInjector>(this));
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

    float getHunger() const { return internalHunger; }
    float getTemperatureStress() const { return internalTempStress; }
    float getHungerThreshold() const { return myStateMachine.getHungerThreshold(); }
    float getTemperatureThreshold() const { return myStateMachine.getTemperatureThreshold(); }

    void feed(float nutrition)
    {
        internalHunger = std::max(0.0f, internalHunger - nutrition);
        myStateMachine.updateSensors(internalFear, internalHunger, internalTempStress);
    }

    const char* getStateName() const
    {
        if (myStateMachine.isScared()) return "Avoiding";
        if (myStateMachine.isHungry()) return "Hungry / seeking prey";
        if (myStateMachine.isSeekingTemperature()) return "Seeking temperature";
        return "Idle / wandering";
    }

    float calculateCurrentVolume(Vector3 geomCenter)
    {
        float currentRadiusSum = 0.0f;
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            currentRadiusSum += Vector3Distance(nodes[i].position, geomCenter);
        }
        float avgRadius = currentRadiusSum / numMembraneNodes;
        return (4.0f / 3.0f) * PI * std::pow(avgRadius, 3);
    }

    void actuate(float dt,
                 const CocciCluster& cocci,
                 const std::vector<BoidState>& flockStates,
                 float currentTemperature,
                 Vector3 temperatureGradient,
                 float targetTemperature)
    {
        for (auto& pf : pendingForces) { pf = Vector3Zero(); }
        
        Vector3 myPos = nodes[0].position;


        internalHunger += dt * 4.0f; 
        
        Vector3 cocciPos = cocci.getCenterPosition();
        float distToCocci = Vector3Distance(cocciPos, myPos);
        internalHunger = Clamp(internalHunger, 0.0f, 100.0f);

        internalFear = std::max(0.0f, internalFear - dt * 15.0f);

        float targetTempStress = Clamp(std::abs(currentTemperature - targetTemperature) * 8.0f, 0.0f, 100.0f);
        if (targetTempStress > internalTempStress)
            internalTempStress = std::min(targetTempStress, internalTempStress + dt * 4.0f);
        else
            internalTempStress = std::max(targetTempStress, internalTempStress - dt * 10.0f);

        myStateMachine.updateSensors(internalFear, internalHunger, internalTempStress);

        float speedMultiplier = 1.0f;
        float stretchFactor = 1.0f;
        float pulseSpeed = 2.2f;
        Vector3 targetHeading = heading;

        if (myStateMachine.isScared())
        {
            speedMultiplier = 3.2f;
            stretchFactor = 2.2f;
            pulseSpeed = 5.0f;
        }
        else if (myStateMachine.isHungry())
        {
            speedMultiplier = 1.1f;
            stretchFactor = 1.10f;
            pulseSpeed = 2.8f;

            bool preySelected = false;
            Vector3 targetFoodPos = Vector3Zero();
            float bestPreyCost = 99999.0f;

            if (distToCocci < searchRadius && distToCocci > 0.033f)
            {
                bestPreyCost = preySelectionCost(distToCocci, 1.0f, 0.0f);
                targetFoodPos = cocciPos;
                preySelected = true;
            }

            for (const auto& boid : flockStates)
            {
                if (!boid.alive) continue;

                float distToBoid = Vector3Distance(boid.position, myPos);
                if (distToBoid >= searchRadius || distToBoid <= 0.033f) continue;

                float escapeCost = preyEscapeCost(boid.position, boid.velocity, myPos);
                float boidCost = preySelectionCost(distToBoid, 0.35f, escapeCost);
                if (boidCost < bestPreyCost)
                {
                    bestPreyCost = boidCost;
                    targetFoodPos = boid.position;
                    preySelected = true;
                }
            }

            if (preySelected)
            {
                Vector3 toPrey = Vector3Subtract(targetFoodPos, myPos);
                // REMOVED: toPrey.y = 0.0f; <-- Let it swim up/down!
                targetHeading = Vector3Normalize(toPrey);
            }
            else
            {
                wanderTimer -= dt;
                if (wanderTimer <= 0.0f)
                {
                    float angle = (float)(GetRandomValue(0, 360)) * DEG2RAD;
                    wanderTargetHeading = { std::cos(angle), 0.0f, std::sin(angle) };
                    wanderTimer = (float)GetRandomValue(1, 3);
                }
                targetHeading = wanderTargetHeading;
            }
        }
        else if (myStateMachine.isSeekingTemperature())
        {
            speedMultiplier = 1.45f;
            stretchFactor = 1.05f;
            pulseSpeed = 2.0f;

            Vector3 tempDir = temperatureGradient;
            if (currentTemperature > targetTemperature)
                tempDir = Vector3Negate(tempDir);

            if (Vector3Length(tempDir) > 0.0001f)
                targetHeading = Vector3Normalize(tempDir);
        }
        else 
        {
            speedMultiplier = 0.5f;
            stretchFactor = 0.7f;
            pulseSpeed = 1.1f;

            wanderTimer -= dt;
            if (wanderTimer <= 0.0f)
            {
                // Generate a true 3D wander heading
                Vector3 randDir = {
                    (float)GetRandomValue(-100, 100),
                    (float)GetRandomValue(-100, 100),
                    (float)GetRandomValue(-100, 100)
                };
                wanderTargetHeading = Vector3Normalize(randDir);
                wanderTimer = (float)GetRandomValue(2, 5); 
            }
            targetHeading = wanderTargetHeading;
        }

        heading = Vector3Lerp(heading, targetHeading, dt * 3.5f);
        heading = Vector3Normalize(heading);

        phase += dt * pulseSpeed; 
        float pulse = std::sin(phase);

        Vector3 geomCenter = Vector3Zero();
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            geomCenter = Vector3Add(geomCenter, nodes[i].position);
        }
        geomCenter = Vector3Scale(geomCenter, 1.0f / (float)numMembraneNodes);

        float currentVolume = calculateCurrentVolume(geomCenter);
        float pressureStiffness = 320.0f; 
        float pressureDelta = targetVolume - currentVolume;
        float internalPressure = std::max(0.0f, pressureDelta * pressureStiffness);

        Vector3 gravityCancellation = {0.0f, 0.0f, 0.0f}; 

        for (size_t i = 0; i < nodes.size(); i++)
        {
            nodes[i].velocity = Vector3Scale(nodes[i].velocity, 0.95f);

            if (i > 0) // Apply pressure to membrane nodes
            {
                // Push outward from the GEOMETRIC center, not the volatile nucleus node
                Vector3 outwardDir = Vector3Normalize(Vector3Subtract(nodes[i].position, geomCenter));
                Vector3 pressureForce = Vector3Scale(outwardDir, internalPressure / numMembraneNodes);
                pendingForces[i] = Vector3Add(pendingForces[i], pressureForce);
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
                    pendingForces[i] = Vector3Add(pendingForces[i], pseudopodThrust);
                    
                    float extensionMagnitude = pulse * 11.6f * stretchFactor * alignment * dt;
                    Vector3 pseudopodStretch = Vector3Scale(dirFromCenter, extensionMagnitude);
                    pendingForces[i] = Vector3Add(pendingForces[i], pseudopodStretch);
                }
            }
            else if (alignment < -0.1f)
            {
                float squeezeMagnitude = 15.0f * speedMultiplier * stretchFactor * std::abs(alignment) * dt;
                Vector3 tailSqueeze = Vector3Scale(heading, squeezeMagnitude);
                pendingForces[i] = Vector3Add(pendingForces[i], tailSqueeze);
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

        Vector3 nucleusFollow = Vector3Scale(heading, 5.0f * speedMultiplier * dt);
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

private:
    float preySelectionCost(float distance, float sizeCost, float escapeCost) const
    {
        return distance * (1.0f + preySizeCostWeight * sizeCost + preyEscapeCostWeight * escapeCost);
    }

    float preyEscapeCost(Vector3 preyPos, Vector3 preyVelocity, Vector3 myPos) const
    {
        Vector3 awayFromHunter = Vector3Subtract(preyPos, myPos);
        awayFromHunter.y = 0.0f;

        Vector3 flatVelocity = preyVelocity;
        flatVelocity.y = 0.0f;

        if (Vector3Length(awayFromHunter) <= 0.033f || Vector3Length(flatVelocity) <= 0.033f)
            return 0.0f;

        float movingAway = Vector3DotProduct(Vector3Normalize(awayFromHunter), Vector3Normalize(flatVelocity));
        return Clamp(movingAway, 0.0f, 1.0f);
    }
};

#endif
