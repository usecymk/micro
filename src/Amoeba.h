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
#include "ObstaclePerception.h"
#include <rlgl.h>

class Amoeba : public PhysicsBody
{
private:
    struct InternalOrganelle
    {
        Vector3 localTargetOffset; // Ideal position relative to geometric center
        Vector3 currentPosition;   // Actual simulated position in world space
        float baseRadius;
        float currentRadius;
        Color color;
        bool isContractile;        // Contractile vacuoles expand and snap shut to pump water
        float pulsePhase;
    };

    float baseRadius;
    float targetVolume;
    float phase;
    Vector3 heading;
    int numMembraneNodes;

    BehaviorStateMachine myStateMachine{50.0f, 50.0f, 20.0f};
    float internalHunger = 0.0f;
    float internalFear = 0.0f;
    float internalTempStress = 0.0f;

    Vector3 wanderTargetHeading;
    float wanderTimer = 0.0f;

    bool    avoidingObstacle = false;
    Vector3 savedIntentHeading = {1.0f, 0.0f, 0.0f};
    float   intentResumeTimer = 0.0f;
    float   obstacleAvoidUrgency = 0.0f;

    float floorY = -5.0f; 

    std::vector<InternalOrganelle> organelles;

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
            return {255, 170, 35, 60};
        } else if (myStateMachine.isSeekingTemperature()) {
            // Temperature stress: violet
            return {145, 105, 255, 55};
        } else {
            // Idle/wandering: calm aqua-green
            return {35, 215, 170, 40};
        }
    }

    Color getMembraneColor() const
    {
        Color color = getIntentBaseColor();
        float hunger01 = internalHunger / 100.0f;
        float tempStress01 = internalTempStress / 100.0f;

        color = lerpColor(color, {255, 120, 45, 80}, hunger01 * 0.30f);

        if (myStateMachine.isHungry()) {
            color = lerpColor(color, {255, 55, 24, 110}, hunger01 * 0.70f);
        } else if (myStateMachine.isSeekingTemperature()) {
            color = lerpColor(color, {255, 135, 55, 100}, tempStress01 * 0.55f);
        }

        return color;
    }

public:
    // --- BEHAVIOR TOGGLES ---
    float searchRadius = 0.0f;
    float preySizeCostWeight = 0.35f;
    float preyEscapeCostWeight = 0.45f;

    std::vector<Vector3> pendingForces;

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
                bodyNodes[i].force = Vector3Add(bodyNodes[i].force, amoeba->pendingForces[i]);
            }
        }
    };

    Amoeba(Vector3 center, float radius = 1.25f, float stiffness = 5.0f, float damping = 2.2f)
    {
        //searchRadius = givenRad;
        baseRadius = radius;
        phase = 0.0f;
        heading = {1.0f, 0.0f, 0.0f}; 
        wanderTargetHeading = heading;
        
        numMembraneNodes = 256; 
        searchRadius = 10.0f; 

        // Internal Nucleus Physics Node
        nodes.push_back(Node(center, 0.15f)); 

        // Distribute membrane surface nodes using Fibonacci Sphere mapping
        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t = (float)i / (numMembraneNodes - 1); 
            float y = 1.0f - (t * 2.0f); 
            float radius_at_y = std::sqrt(1.0f - y * y);
            float theta = PI * (1.0f + std::sqrt(5.0f)) * i; 
            
            float x = std::cos(theta) * radius_at_y;
            float z = std::sin(theta) * radius_at_y;

            Vector3 offset = Vector3Scale({x, y, z}, radius);
            nodes.push_back(Node(Vector3Add(center, offset), 0.04f)); 
        }

        // Structural Core Springs (Nucleus to Membrane)
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            addSpring(0, i, stiffness * 0.5f, damping); 
        }

        // Surface Cross-Stitching Springs
        float expected_dist = std::sqrt(4.0f * PI * radius * radius / numMembraneNodes);
        float threshold = expected_dist * 1.6f; 

        for (int i = 1; i <= numMembraneNodes; i++)
        {
            for (int j = i + 1; j <= numMembraneNodes; j++)
            {
                float d = Vector3Distance(nodes[i].position, nodes[j].position);
                if (d < threshold)
                {
                    addSpring(i, j, stiffness * 0.75f, damping);
                }
            }
        }

        targetVolume = (4.0f / 3.0f) * PI * std::pow(radius, 3);
        pendingForces.resize(nodes.size(), Vector3Zero());
        addForceGenerator(std::make_unique<AmoebaForceInjector>(this));

        // Initialize Procedural Organelles
        // 1. Large centralized Nucleus
        organelles.push_back({ {0.0f, 0.05f, -0.02f}, center, radius * 0.28f, radius * 0.28f, {75, 45, 110, 180}, false, 0.0f });
        
        // 2. Water-regulating Contractile Vacuole
        organelles.push_back({ {-0.15f, -0.05f, 0.12f}, center, radius * 0.18f, radius * 0.18f, {110, 190, 255, 160}, true, 0.0f });
        
        // 3. Smaller metabolic granules / food vacuoles scattered around the cytoplasm
        for (int i = 0; i < 8; i++)
        {
            float rx = ((float)GetRandomValue(-100, 100) * 0.0025f) * radius;
            float ry = ((float)GetRandomValue(-100, 100) * 0.0025f) * radius;
            float rz = ((float)GetRandomValue(-100, 100) * 0.0025f) * radius;
            float randSize = radius * (0.04f + (float)GetRandomValue(0, 4) * 0.015f);
            
            Color organelleColor = (GetRandomValue(0, 1) == 0) 
                ? Color{130, 150, 70, 140}  // Food/Nutrient storage (greenish-brown)
                : Color{175, 160, 120, 130}; // Fat droplets / general granules
                
            organelles.push_back({ {rx, ry, rz}, center, randSize, randSize, organelleColor, false, (float)GetRandomValue(0, 360) });
        }
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

    Vector3 getHeading() const { return heading; }
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
        if (obstacleAvoidUrgency > 0.12f) return "Avoiding obstacle";
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
                 const std::vector<std::unique_ptr<CocciCluster>>& cocciClusters, // Changed signature
                 const std::vector<BoidState>& flockStates,
                 float currentTemperature,
                 Vector3 temperatureGradient,
                 float targetTemperature,
                 float comfortBand,
                 const std::vector<Obstacle>& obstacles)
    {
        for (auto& pf : pendingForces) { pf = Vector3Zero(); }
        
        Vector3 myPos = nodes[0].position;

        internalHunger += dt * 4.0f; 
        internalHunger = Clamp(internalHunger, 0.0f, 100.0f);

        if (internalHunger >= 95.0f) 
        {
            starvationTimer += dt;
        } else {
            starvationTimer = std::max(0.0f, starvationTimer - dt * 2.0f);
        }

        if (starvationTimer > 10.0f) 
        {
            isDead = true;
        }
        
        if (isDead) return;

        internalFear = std::max(0.0f, internalFear - dt * 15.0f);

        float targetTempStress = Clamp(std::abs(currentTemperature - targetTemperature) * 12.0f, 0.0f, 100.0f);
        if (targetTempStress > internalTempStress)
            internalTempStress = std::min(targetTempStress, internalTempStress + dt * 4.0f);
        else
            internalTempStress = std::max(targetTempStress, internalTempStress - dt * 3.0f);

        myStateMachine.updateSensors(internalFear, internalHunger, internalTempStress);

        float speedMultiplier = 1.0f;
        float stretchFactor = 1.0f;
        float pulseSpeed = 2.2f;
        Vector3 targetHeading = heading;

        if (myStateMachine.isScared())
        {
            speedMultiplier = 2.0f;
            stretchFactor = 2.4f;
            pulseSpeed = 4.8f;
        }
        else if (myStateMachine.isHungry())
        {
            speedMultiplier = 1.0f;
            stretchFactor = 1.35f;
            pulseSpeed = 2.6f;

            bool preySelected = false;
            Vector3 targetFoodPos = Vector3Zero();
            float bestPreyCost = 99999.0f;

            // Loop through and evaluate all available Cocci clusters
            for (const auto& cocci : cocciClusters)
            {
                Vector3 cocciPos = cocci->getCenterPosition();
                float distToCocci = Vector3Distance(cocciPos, myPos);
                if (distToCocci < searchRadius && distToCocci > 0.033f)
                {
                    float cocciCost = preySelectionCost(distToCocci, 1.0f, 0.0f);
                    if (cocciCost < bestPreyCost)
                    {
                        bestPreyCost = cocciCost;
                        targetFoodPos = cocciPos;
                        preySelected = true;
                    }
                }
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
            float deviation = currentTemperature - targetTemperature;
            float band = std::max(comfortBand, 0.5f);

            if (std::fabs(deviation) < band)
            {
                speedMultiplier = 0.4f;
                stretchFactor = 0.85f;
                pulseSpeed = 1.4f;

                wanderTimer -= dt;
                if (wanderTimer <= 0.0f)
                {
                    Vector3 randDir = {
                        (float)GetRandomValue(-100, 100),
                        (float)GetRandomValue(-100, 100),
                        (float)GetRandomValue(-100, 100)
                    };
                    wanderTargetHeading = Vector3Normalize(randDir);
                    wanderTimer = (float)GetRandomValue(2, 4);
                }
                targetHeading = wanderTargetHeading;
            }
            else if (Vector3Length(temperatureGradient) > 0.0001f)
            {
                speedMultiplier = 0.72f;
                stretchFactor = 0.95f;
                pulseSpeed = 1.7f;

                Vector3 tempDir = (deviation < 0.0f)
                    ? temperatureGradient
                    : Vector3Negate(temperatureGradient);
                targetHeading = Vector3Normalize(tempDir);
            }
        }
        else 
        {
            speedMultiplier = 0.25f;
            stretchFactor = 0.7f;
            pulseSpeed = 1.1f;

            wanderTimer -= dt;
            if (wanderTimer <= 0.0f)
            {
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

        ObstacleSenseParams obsParams;
        obsParams.senseRadius     = 6.5f;
        obsParams.attentionRadius = 3.0f;
        obsParams.criticalRadius  = 0.55f;

        AttentionMode attention = AttentionMode::WANDER;
        if (myStateMachine.isScared())
            attention = AttentionMode::FEARFUL;
        else if (myStateMachine.isHungry())
            attention = AttentionMode::HUNGRY;
        applySelectiveAttention(obsParams, attention);

        // Sense physical environment obstacles
        ObstacleSenseResult obsSense = senseObstacles(
            myPos, heading, obstacles, obsParams, 0.12f, attention);
        obstacleAvoidUrgency = obsSense.urgency;

        if (obsSense.detected && obsSense.urgency > 0.12f)
        {
            if (!avoidingObstacle)
            {
                savedIntentHeading = targetHeading;
                avoidingObstacle = true;
            }

            if (obsSense.urgency > 0.65f)
                targetHeading = obsSense.avoidDirection;
            else
                targetHeading = Vector3Normalize(
                    Vector3Lerp(targetHeading, obsSense.avoidDirection, obsSense.urgency));

            speedMultiplier *= 1.0f + obsSense.urgency * 0.45f;
            stretchFactor   *= 1.0f + obsSense.urgency * 0.25f;
        }
        else if (avoidingObstacle)
        {
            avoidingObstacle = false;
            intentResumeTimer = 0.5f;
        }

        if (intentResumeTimer > 0.0f)
        {
            intentResumeTimer -= dt;
            float resumeBlend = Clamp(intentResumeTimer / 0.5f, 0.0f, 1.0f);
            targetHeading = Vector3Normalize(
                Vector3Lerp(savedIntentHeading, targetHeading, 1.0f - resumeBlend));
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
        float pressureStiffness = 380.0f; 
        float pressureDelta = targetVolume - currentVolume;
        float internalPressure = std::max(0.0f, pressureDelta * pressureStiffness);

        for (size_t i = 0; i < nodes.size(); i++)
        {
            nodes[i].velocity = Vector3Scale(nodes[i].velocity, 0.93f);

            if (i > 0) 
            {
                Vector3 outwardDir = Vector3Normalize(Vector3Subtract(nodes[i].position, geomCenter));
                Vector3 pressureForce = Vector3Scale(outwardDir, internalPressure / numMembraneNodes);
                pendingForces[i] = Vector3Add(pendingForces[i], pressureForce);
            }
        }

        // ── Calibrated Cytoplasmic Streaming Engine ──────────────────────────
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            Vector3 dirFromCenter = Vector3Normalize(Vector3Subtract(nodes[i].position, nodes[0].position));
            float alignment = Vector3DotProduct(dirFromCenter, heading);

            // 1. ACTIVE FRONT (Tuned from 72.0f -> 13.0f, and 32.0f -> 6.5f)
            if (alignment > 0.35f) 
            {
                if (pulse > 0.0f)
                {
                    float thrustMagnitude = pulse * 13.0f * speedMultiplier * std::pow(alignment, 2) * dt;
                    Vector3 pseudopodThrust = Vector3Scale(heading, thrustMagnitude);
                    pendingForces[i] = Vector3Add(pendingForces[i], pseudopodThrust);
                    
                    float extensionMagnitude = pulse * 6.5f * stretchFactor * alignment * dt;
                    Vector3 pseudopodStretch = Vector3Scale(dirFromCenter, extensionMagnitude);
                    pendingForces[i] = Vector3Add(pendingForces[i], pseudopodStretch);
                }
            }
            // 2. LATERAL SQUEEZE (Tuned from 24.0f -> 4.5f)
            else if (std::abs(alignment) <= 0.35f)
            {
                float sideSqueeze = (pulse * 0.5f + 0.5f) * 4.5f * speedMultiplier * (0.35f - std::abs(alignment)) * dt;
                Vector3 inwardSqueeze = Vector3Scale(dirFromCenter, -sideSqueeze);
                pendingForces[i] = Vector3Add(pendingForces[i], inwardSqueeze);
            }
            // 3. UROPOD CONTRACTION (Tuned from 42.0f -> 8.0f)
            else if (alignment < -0.2f)
            {
                float squeezeMagnitude = 8.0f * speedMultiplier * stretchFactor * std::abs(alignment) * dt;
                Vector3 tailSqueeze = Vector3Scale(heading, squeezeMagnitude);
                pendingForces[i] = Vector3Add(pendingForces[i], tailSqueeze);
            }

            // Substrate frictional grounding physics
            if (nodes[i].position.y < floorY + 0.0667f)
            {
                if (alignment > 0.4f && pulse > 0.1f)
                {
                    nodes[i].velocity.x *= 0.005f;
                    nodes[i].velocity.z *= 0.005f;
                }
                else 
                {
                    nodes[i].velocity.x *= 0.90f;
                    nodes[i].velocity.z *= 0.90f;
                }
            }
        }

        // Nucleus node tracking pull scaled down proportionally (1.5f -> 0.32f)
        Vector3 nucleusFollow = Vector3Scale(heading, 0.32f * speedMultiplier * dt);
        nodes[0].velocity = Vector3Add(nodes[0].velocity, nucleusFollow / 5.0f);
        
        // Update Internal Procedural Organelles
        for (auto& org : organelles)
        {
            Vector3 targetWorldPos = Vector3Add(geomCenter, org.localTargetOffset);
            
            if (org.isContractile)
            {
                org.pulsePhase += dt * 1.6f;
                float wave = std::sin(org.pulsePhase);
                if (wave < -0.85f) 
                {
                    org.currentRadius = org.baseRadius * 0.3f;
                }
                else 
                {
                    org.currentRadius = org.baseRadius * (1.0f + (wave + 1.0f) * 0.35f);
                }
            }

            org.currentPosition = Vector3Lerp(org.currentPosition, targetWorldPos, dt * 4.5f);
        }

        float speed = Vector3Length(nodes[0].velocity);
        if (speed > 0.05f)
        {
            nodes[0].velocity = Vector3Scale(Vector3Normalize(nodes[0].velocity), 0.05f);
        }
    }

    void draw(Shader membraneShader, Vector3 cameraPos, float time, bool debugOverlay = false)
    {
        if (nodes.empty() || isDead) return;

        int numNodes = (int)nodes.size();

        // 1. RENDER ORGANELLES (Unshaded, drawn before the membrane)
        // for (const auto& org : organelles)
        // {
        //     DrawSphere(org.currentPosition, org.currentRadius, org.color);
        //     DrawSphere(org.currentPosition, org.currentRadius * 0.4f, 
        //                ColorAlpha(org.color, Clamp(org.color.a / 255.0f + 0.2f, 0.0f, 1.0f)));
        // }

        // 2. PREPARE THE SHADER
        int viewPosLoc = GetShaderLocation(membraneShader, "viewPos");
        int timeLoc = GetShaderLocation(membraneShader, "time");
        
        SetShaderValue(membraneShader, viewPosLoc, &cameraPos, SHADER_UNIFORM_VEC3);
        SetShaderValue(membraneShader, timeLoc, &time, SHADER_UNIFORM_FLOAT);

        Color faceColor = getMembraneColor(); 

        // Calculate geometric center for smooth normal generation
        Vector3 geomCenter = Vector3Zero();
        for (int i = 1; i < numNodes; i++) {
            geomCenter = Vector3Add(geomCenter, nodes[i].position);
        }
        geomCenter = Vector3Scale(geomCenter, 1.0f / (numNodes - 1));

        std::vector<std::vector<bool>> connected(numNodes, std::vector<bool>(numNodes, false));
        for (const auto& s : springs)
        {
            int u = s.nodeA - &nodes[0];
            int v = s.nodeB - &nodes[0];
            if (u >= 0 && u < numNodes && v >= 0 && v < numNodes) {
                connected[u][v] = true;
                connected[v][u] = true;
            }
        }

        // 3. DRAW MEMBRANE WITH CUSTOM RLGL BATCH
        BeginBlendMode(BLEND_ALPHA); // Ensure proper transparency rendering
        BeginShaderMode(membraneShader);

        rlBegin(RL_TRIANGLES);
        rlColor4ub(faceColor.r, faceColor.g, faceColor.b, faceColor.a);

        for (int u = 1; u < numNodes; u++)
        {
            for (int v = u + 1; v < numNodes; v++)
            {
                if (!connected[u][v]) continue;

                for (int w = v + 1; w < numNodes; w++)
                {
                    if (connected[u][w] && connected[v][w])
                    {
                        // Calculate smooth normals radiating from the geometric center
                        Vector3 nU = Vector3Normalize(Vector3Subtract(nodes[u].position, geomCenter));
                        Vector3 nV = Vector3Normalize(Vector3Subtract(nodes[v].position, geomCenter));
                        Vector3 nW = Vector3Normalize(Vector3Subtract(nodes[w].position, geomCenter));

                        // Explicitly bind the color right before generating the vertices
                        rlColor4ub(faceColor.r, faceColor.g, faceColor.b, faceColor.a);

                        // Pass 1: Outer Face
                        rlNormal3f(nU.x, nU.y, nU.z); rlVertex3f(nodes[u].position.x, nodes[u].position.y, nodes[u].position.z);
                        rlNormal3f(nV.x, nV.y, nV.z); rlVertex3f(nodes[v].position.x, nodes[v].position.y, nodes[v].position.z);
                        rlNormal3f(nW.x, nW.y, nW.z); rlVertex3f(nodes[w].position.x, nodes[w].position.y, nodes[w].position.z);

                        // Pass 2: Inner Face (Reversed winding order)
                        rlNormal3f(nU.x, nU.y, nU.z); rlVertex3f(nodes[u].position.x, nodes[u].position.y, nodes[u].position.z);
                        rlNormal3f(nW.x, nW.y, nW.z); rlVertex3f(nodes[w].position.x, nodes[w].position.y, nodes[w].position.z);
                        rlNormal3f(nV.x, nV.y, nV.z); rlVertex3f(nodes[v].position.x, nodes[v].position.y, nodes[v].position.z);
                    }
                }
            }
        }
        
        rlEnd();
        EndShaderMode();
        EndBlendMode();

        // 4. DEBUG OVERLAY
        if (debugOverlay)
        {
            Vector3 com = nodes[0].position;
            DrawLine3D(com, Vector3Add(com, Vector3Scale(heading, 2.5f)), LIME);
            DrawSphere(Vector3Add(com, Vector3Scale(heading, 2.5f)), 0.12f, LIME);
        }
    }

private:

    float starvationTimer = 0.0f;
    bool isDead = false;

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