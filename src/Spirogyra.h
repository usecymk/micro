#ifndef MICRO3D_SPIROGYRA_H
#define MICRO3D_SPIROGYRA_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"
#include "ForceGenerator.h"
#include "BehaviorStateMachine.h"


struct SpirogyraDrive
{
    float   undulationScale = 1.0f;       
    float   cruiseAccel     = 4.0f;       
    Vector3 biasDir         = {0,0,0};    
    float   biasStrength    = 0.0f;       
};


class SpirogyraActuator : public ForceGenerator
{
    const SpirogyraDrive *drive;
    float time        = 0.0f;
    float wanderAngle = 0.0f;
    float pitchAngle  = 0.0f;
    float wanderTimer = 0.0f;

public:
    explicit SpirogyraActuator(const SpirogyraDrive *d) : drive(d) {}

    void apply(PhysicsBody &body, float dt) override
    {
        auto &ns = body.getNodes();
        int n = (int)ns.size();
        if (n < 2) return;

        const float undScale  = drive ? drive->undulationScale : 1.0f;
        const float cruise    = drive ? drive->cruiseAccel     : 4.0f;
        const Vector3 biasDir  = drive ? drive->biasDir          : Vector3Zero();
        const float biasStr   = drive ? drive->biasStrength    : 0.0f;

        time += dt;

        Vector3 axis = Vector3Subtract(ns[n - 1].position, ns[0].position);
        if (Vector3Length(axis) < 1e-5f) axis = {1.0f, 0.0f, 0.0f};
        axis = Vector3Normalize(axis);

        Vector3 side = Vector3CrossProduct(axis, {0.0f, 1.0f, 0.0f});
        if (Vector3Length(side) < 1e-4f)
            side = Vector3CrossProduct(axis, {1.0f, 0.0f, 0.0f});
        side = Vector3Normalize(side);

        const float freq      = 2.4f;
        const float phase     = 0.55f;
        const float waveAccel = 16.0f * undScale; 

        for (int i = 0; i < n; i++)
        {
            float wave     = std::sin(freq * time - i * phase);
            float envelope = 0.6f + 0.4f * std::sin(PI * (float)i / (float)(n - 1));
            Vector3 F = Vector3Scale(side, waveAccel * wave * envelope * ns[i].mass);
            ns[i].force = Vector3Add(ns[i].force, F);
        }

        wanderTimer -= dt;
        if (wanderTimer <= 0.0f)
        {
            wanderAngle += (float)GetRandomValue(-45, 45) * DEG2RAD;
            pitchAngle   = (float)GetRandomValue(-30, 30) * DEG2RAD;
            wanderTimer  = (float)GetRandomValue(15, 35) / 10.0f;
        }

        float cp = std::cos(pitchAngle);
        Vector3 swimDir = {
            std::cos(wanderAngle) * cp,
            std::sin(pitchAngle),
            std::sin(wanderAngle) * cp,
        };
        for (auto &nd : ns)
            nd.force = Vector3Add(nd.force, Vector3Scale(swimDir, cruise * nd.mass));

        if (biasStr > 0.0f && Vector3Length(biasDir) > 1e-5f)
        {
            Vector3 b = Vector3Normalize(biasDir);
            for (auto &nd : ns)
                nd.force = Vector3Add(nd.force, Vector3Scale(b, biasStr * nd.mass));
        }

        const float bobAccel = 1.5f;
        float bob = std::sin(time * 0.8f);
        for (auto &nd : ns)
            nd.force.y += bobAccel * bob * nd.mass;
    }
};


class Spirogyra : public PhysicsBody
{
public:
    Spirogyra(Vector3 startPos    = {0.0f, 2.0f, 0.0f},
              int     numNodes    = 16,
              float   segmentLen  = 0.32f,
              float   stiffness   = 55.0f,
              float   damping     = 2.2f)
        : numNodes(numNodes)
    {
        nodes.reserve(numNodes);

        Vector3 axis = {1.0f, 0.0f, 0.0f};
        for (int i = 0; i < numNodes; i++)
        {
            float t      = (float)i / (float)(numNodes - 1);
            float curve  = std::sin(t * PI) * 0.5f;
            Vector3 pos  = Vector3Add(startPos, Vector3Scale(axis, i * segmentLen));
            pos.z       += curve;
            nodes.push_back(Node(pos, 0.04f, 0.04f / 1000.0f)); // neutral buoyancy
        }

        for (int i = 0; i < numNodes - 1; i++)
            addSpring(i, i + 1, stiffness, damping);

        for (int i = 0; i < numNodes - 2; i++)
            addSpring(i, i + 2, stiffness * 0.22f, damping * 0.8f);

        heading = axis;

        addForceGenerator(std::make_unique<SpirogyraActuator>(&drive));
    }

    void setEnvironment(float floorY, float ceilY, float seekRadius)
    {
        envFloorY  = floorY;
        envCeilY   = ceilY;
        envSeekR   = seekRadius;
    }

    void update(float dt)
    {
        actuate(dt);

        const float maxStep = 1.0f / 120.0f;
        if (dt > 0.1f) dt = 0.1f;

        int steps = (int)std::ceil(dt / maxStep);
        if (steps < 1) steps = 1;
        float h = dt / (float)steps;

        for (int i = 0; i < steps; i++)
        {
            updateHeading();
            updatePhysicsImplicit(h);
        }
    }

    void onWallHit(Vector3 awayDir)
    {
        internalFear = Clamp(internalFear + 60.0f, 0.0f, 100.0f);
        if (Vector3Length(awayDir) > 1e-5f)
            fleeDir = Vector3Normalize(awayDir);
    }

    bool  isScared()  const { return bsm.isScared(); }
    bool  isHungry()  const { return bsm.isHungry(); }
    float getHunger() const { return internalHunger; }
    float getFear()   const { return internalFear; }

    void draw() const
    {
        if (nodes.empty()) return;

        float fear   = internalFear   / 100.0f;
        float hunger = internalHunger / 100.0f;
        Color wallCol = {
            (unsigned char)Clamp(70.0f  + fear * 150.0f + hunger * 60.0f, 0.0f, 255.0f),
            (unsigned char)Clamp(190.0f - fear * 90.0f,                   0.0f, 255.0f),
            (unsigned char)Clamp(110.0f - fear * 70.0f,                   0.0f, 255.0f),
            235};
        const Color jointCol = { 40, 150,  80, 255}; 
        const float tube     = 0.11f;

        for (int i = 0; i < (int)nodes.size() - 1; i++)
        {
            DrawCapsule(nodes[i].position, nodes[i + 1].position, tube, 8, 4, wallCol);
            if (i % 2 == 0)
                DrawSphere(nodes[i].position, tube * 1.15f, jointCol);
        }
        DrawSphere(nodes.front().position, tube * 1.15f, jointCol);
        DrawSphere(nodes.back().position,  tube * 1.15f, jointCol);

        drawChloroplast(tube);
    }

    Vector3 getCenterPosition() const
    {
        if (nodes.empty()) return Vector3Zero();
        Vector3 sum = Vector3Zero();
        for (const auto &n : nodes)
            sum = Vector3Add(sum, n.position);
        return Vector3Scale(sum, 1.0f / (float)nodes.size());
    }

    Vector3 getHeading() const { return heading; }

private:
    int     numNodes;
    Vector3 heading = {1.0f, 0.0f, 0.0f};

    BehaviorStateMachine bsm;
    SpirogyraDrive       drive;            
    float internalHunger     = 0.0f;       
    float internalFear       = 0.0f;       
    float internalTempStress = 0.0f;       
    Vector3 fleeDir          = {0.0f, 0.0f, 0.0f};

    float envFloorY = 0.0f;
    float envCeilY  = 5.0f;
    float envSeekR  = 8.0f;

    void actuate(float dt)
    {
        Vector3 c = getCenterPosition();

        internalHunger += dt * 4.0f;
        float lightBand = envFloorY + (envCeilY - envFloorY) * 0.6f;
        if (c.y > lightBand)
            internalHunger -= dt * 30.0f;
        internalHunger = Clamp(internalHunger, 0.0f, 100.0f);

        internalFear = std::max(0.0f, internalFear - dt * 15.0f);

        float distXZ = Vector3Length({c.x, 0.0f, c.z});
        if (distXZ > envSeekR) internalTempStress += dt * 12.0f;
        else                   internalTempStress -= dt * 8.0f;
        internalTempStress = Clamp(internalTempStress, 0.0f, 100.0f);

        bsm.updateSensors(internalFear, internalHunger, internalTempStress);

        drive.undulationScale = 1.0f;
        drive.cruiseAccel     = 4.0f;
        drive.biasDir         = {0.0f, 0.0f, 0.0f};
        drive.biasStrength    = 0.0f;

        if (bsm.isScared())
        {
            drive.undulationScale = 2.0f;
            drive.cruiseAccel     = 9.0f;
            drive.biasDir         = fleeDir;
            drive.biasStrength    = 8.0f;
        }
        else if (bsm.isHungry())
        {
            drive.undulationScale = 1.3f;
            drive.cruiseAccel     = 6.0f;
            drive.biasDir         = {0.0f, 1.0f, 0.0f};
            drive.biasStrength    = 5.0f;
        }
        else if (bsm.isSeekingTemperature())
        {
            Vector3 toCenter = {-c.x, 0.0f, -c.z};
            if (Vector3Length(toCenter) > 1e-4f)
            {
                drive.biasDir      = Vector3Normalize(toCenter);
                drive.biasStrength = 5.0f;
            }
        }
    }

    void updateHeading()
    {
        Vector3 a = Vector3Subtract(nodes.back().position, nodes.front().position);
        if (Vector3Length(a) < 1e-5f) return;
        a = Vector3Normalize(a);
        heading = Vector3Normalize(Vector3Lerp(heading, a, 0.1f));
    }

    void drawChloroplast(float tube) const
    {
        const Color spiralCol = { 30, 120,  50, 255};
        const float coilR     = tube * 1.05f;
        const float turns     = 2.5f;          
        const int   perSeg    = 5;             

        Vector3 up = {0.0f, 1.0f, 0.0f};
        float arc  = 0.0f;

        for (int i = 0; i < (int)nodes.size() - 1; i++)
        {
            Vector3 a = nodes[i].position;
            Vector3 b = nodes[i + 1].position;
            Vector3 tan = Vector3Subtract(b, a);
            float segLen = Vector3Length(tan);
            if (segLen < 1e-5f) continue;
            tan = Vector3Normalize(tan);

            Vector3 n1 = Vector3CrossProduct(tan, up);
            if (Vector3Length(n1) < 1e-4f)
                n1 = Vector3CrossProduct(tan, {1.0f, 0.0f, 0.0f});
            n1 = Vector3Normalize(n1);
            Vector3 n2 = Vector3Normalize(Vector3CrossProduct(tan, n1));

            for (int s = 0; s < perSeg; s++)
            {
                float f     = (float)s / (float)perSeg;
                float angle = arc + f * turns * 2.0f * PI / (float)(numNodes - 1);
                Vector3 radial = Vector3Add(
                    Vector3Scale(n1, std::cos(angle) * coilR),
                    Vector3Scale(n2, std::sin(angle) * coilR));
                Vector3 p = Vector3Add(Vector3Lerp(a, b, f), radial);
                DrawSphere(p, tube * 0.32f, spiralCol);
            }
            arc += turns * 2.0f * PI / (float)(numNodes - 1);
        }
    }
};

#endif // MICRO3D_SPIROGYRA_H
