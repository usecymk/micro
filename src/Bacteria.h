#ifndef MICRO3D_BACTERIA_H
#define MICRO3D_BACTERIA_H

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"
#include "BehaviorStateMachine.h"

class Bacteria : public PhysicsBody
{
public:
    enum : int {
        BODY_NODES = 4,
        FLAG_NODES = 14,
        TOTAL_NODES = BODY_NODES + FLAG_NODES,
    };

    BehaviorStateMachine bsm;
    float scale = 1.0f;

    Bacteria(Vector3 spawnPos = {0.0f, 2.0f, 0.0f},
             float stiffness = 60.0f,
             float damping = 2.0f)
    {
        float sx = spawnPos.x, sy = spawnPos.y, sz = spawnPos.z;
        float k  = stiffness, d  = damping;

        nodes.reserve(TOTAL_NODES);

        //body nodes – stiff pill along +Z (1/5 scale)
        nodes.push_back(Node({sx, sy, sz - 0.080f}));   //0 front
        nodes.push_back(Node({sx, sy, sz - 0.026f}));   //1
        nodes.push_back(Node({sx, sy, sz + 0.026f}));   //2
        nodes.push_back(Node({sx, sy, sz + 0.080f}));   //3 back

        //flagellum nodes trailing in +Z
        for (int i = 0; i < FLAG_NODES; i++) {
            nodes.push_back(Node({sx, sy, sz + 0.080f + (i + 1) * 0.0045f}));
        }

        //body springs
        addSpring(0, 1, k, d);
        addSpring(1, 2, k, d);
        addSpring(2, 3, k, d);
        addSpring(0, 2, k * 0.15f, d);
        addSpring(1, 3, k * 0.15f, d);
        addSpring(0, 3, k * 0.03f, d);

        //flagellum tapered stiffness (stiffer at base, softer at tip)
        for (int i = 0; i < FLAG_NODES; i++)
        {
            float t = (float)i / (float)(FLAG_NODES - 1);
            float fk_i = k * (0.25f - t * 0.15f);
            float fd_i = d * (0.25f - t * 0.18f);
            addSpring(BODY_NODES - 1 + i, BODY_NODES + i, fk_i, fd_i);
        }

        initOrientation();
    }

    void update(float dt)
    {
        accumulateExternalForces(dt);
        updateHeading();
        bsm.update(dt);
        applyMotorControls();
        animateFlagellum(dt);
        updatePhysicsWithDrag(dt);
    }

    void draw(bool debugOverlay = false)
    {
        if (!bsm.state.alive) {
            return;
        }
        auto &ns = getNodes();

        const float radius = 0.036f * scale;
        const Color bodyCol = {80, 220, 120, 255};
        for (int i = 0; i < BODY_NODES - 1; i++) {
            DrawCapsule(ns[i].position, ns[i + 1].position, radius, 6, 4, bodyCol);
        }
        DrawSphere(ns[0].position, radius, bodyCol);
        DrawSphere(ns[BODY_NODES-1].position, radius, bodyCol);

        for (int i = BODY_NODES - 1; i < TOTAL_NODES - 1; i++) {
            float t = (float)(i - (BODY_NODES - 1)) / (float)FLAG_NODES;
            float thickness = 0.011f * scale * (1.0f - t * 0.75f);
            DrawCapsule(ns[i].position, ns[i + 1].position, thickness, 4, 2, {100, 210, 255, 255});
        }

        if (debugOverlay) {
            Vector3 com = getCenterOfMass();
            DrawLine3D(com, Vector3Add(com, Vector3Scale(heading, 0.16f)), YELLOW);
            DrawSphere(Vector3Add(com, Vector3Scale(heading, 0.16f)), 0.01f, YELLOW);
            Vector3 worldUp    = {0.0f, 1.0f, 0.0f};
            Vector3 localRight = Vector3Normalize(Vector3CrossProduct(heading, worldUp));
            DrawLine3D(com, Vector3Add(com, Vector3Scale(localRight, 0.08f)), RED);
        }
    }

    Vector3 getCenterOfMass()
    {
        Vector3 com = Vector3Zero();
        for (int i = 0; i < BODY_NODES; i++) {
            com = Vector3Add(com, nodes[i].position);
        }
        return Vector3Scale(com, 1.0f / BODY_NODES);
    }

    Vector3 getHeading() const { return heading; }

    void onWallHit(Vector3 awayDir)
    {
        bsm.onWallHit(awayDir);
    }

    void reset(Vector3 spawnPos = {0.0f, 2.0f, 0.0f})
    {
        nodes.clear();
        springs.clear();
        time = 0.0f;
        heading = {0.0f, 0.0f, -1.0f};
        bsm.resetBehavior();

        float sx = spawnPos.x, sy = spawnPos.y, sz = spawnPos.z;
        float k = 60.0f, d = 2.0f;
        nodes.reserve(TOTAL_NODES);
        nodes.push_back(Node({sx, sy, sz - 0.080f}));
        nodes.push_back(Node({sx, sy, sz - 0.026f}));
        nodes.push_back(Node({sx, sy, sz + 0.026f}));
        nodes.push_back(Node({sx, sy, sz + 0.080f}));
        for (int i = 0; i < FLAG_NODES; i++) {
            nodes.push_back(Node({sx, sy, sz + 0.080f + (i + 1) * 0.0045f}));
        }
        addSpring(0, 1, k, d);
        addSpring(1, 2, k, d);
        addSpring(2, 3, k, d);
        addSpring(0, 2, k * 0.15f, d);
        addSpring(1, 3, k * 0.15f, d);
        addSpring(0, 3, k * 0.03f, d);
        for (int i = 0; i < FLAG_NODES; i++) {
            float t = (float)i / (float)(FLAG_NODES - 1);
            float fk_i = k * (0.25f - t * 0.15f);
            float fd_i = d * (0.25f - t * 0.18f);
            addSpring(BODY_NODES - 1 + i, BODY_NODES + i, fk_i, fd_i);
        }
        initOrientation();
    }

private:
    float time = 0.0f;
    float drag = 0.985f;
    Vector3 heading = {0.0f, 0.0f, -1.0f};

    void initOrientation()
    {
        Vector3 bf = Vector3Subtract(nodes[0].position, nodes[BODY_NODES - 1].position);
        if (Vector3Length(bf) > 1e-4f) {
            heading = Vector3Normalize(bf);
        }
        bsm.setHeading(heading, atan2f(heading.x, heading.z));
    }

    void updateHeading()
    {
        Vector3 bf = Vector3Subtract(nodes[0].position, nodes[BODY_NODES - 1].position);
        if (Vector3Length(bf) > 1e-4f) {
            heading = Vector3Normalize(bf);
        }
        bsm.setHeading(heading, atan2f(heading.x, heading.z));
    }

    void applyMotorControls()
    {
        Vector3 thrustDir = heading;
        if (bsm.behavior == Behavior::ESCAPE && Vector3Length(bsm.getFleeDirection()) > 0.1f) {
            Vector3 blended = Vector3Add(heading, Vector3Scale(bsm.getFleeDirection(), 2.0f));
            if (Vector3Length(blended) > 1e-4f) {
                thrustDir = Vector3Normalize(blended);
            }
        }

        float thrust = bsm.swimMC.getThrust();
        for (int i = 0; i < BODY_NODES; i++) {
            nodes[i].force = Vector3Add(nodes[i].force, Vector3Scale(thrustDir, thrust));
        }

        Vector3 worldUp = {0.0f, 1.0f, 0.0f};
        Vector3 localRight = Vector3Normalize(Vector3CrossProduct(heading, worldUp));
        float net = bsm.turnMC.right - bsm.turnMC.left;
        nodes[0].force = Vector3Add(nodes[0].force, Vector3Scale(localRight, net * 0.2f));
        nodes[1].force = Vector3Add(nodes[1].force, Vector3Scale(localRight, net * 0.06f));
        nodes[0].force.y += bsm.turnMC.pitch * 0.18f;
    }

    void animateFlagellum(float dt)
    {
        time += dt;

        Vector3 bf = Vector3Subtract(nodes[BODY_NODES - 1].position, nodes[0].position);
        if (Vector3Length(bf) < 1e-4f) {
            return;
        }
        Vector3 side = Vector3Normalize(
            Vector3CrossProduct(Vector3Normalize(bf), {0.0f, 1.0f, 0.0f}));

        float curAmp = bsm.swimMC.getAmplitude();
        float curFreq = bsm.swimMC.getFrequency();
        const float phase = 0.45f;

        for (int i = 0; i < FLAG_NODES; i++)
        {
            float t = (float)i / (float)(FLAG_NODES - 1);
            float amp = curAmp * (1.0f + t * 0.4f);
            float fMag = 2.0f * amp * sinf(curFreq * time - i * phase);
            float forceScale = 1.0f - t * 0.55f;
            nodes[BODY_NODES + i].force = Vector3Add(
                nodes[BODY_NODES + i].force,
                Vector3Scale(side, fMag * forceScale));
        }
    }

    void updatePhysicsWithDrag(float dt)
    {
        for (auto &s : springs) {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f) continue;
            Vector3 dhat = Vector3Normalize(d);
            float fs = s.stiffness * (L - s.rest_length);
            float vrel = Vector3DotProduct(
                Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), dhat);
            float fd = s.damping * vrel;
            Vector3 F = Vector3Scale(dhat, -(fs + fd));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add   (s.nodeB->force, F);
        }

        for (auto &n : nodes) {
            n.acceleration = Vector3Scale(n.force, 1.0f / n.mass);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.velocity = Vector3Scale(n.velocity, drag);
            n.position  = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        }
    }
};

#endif // MICRO3D_BACTERIA_H
