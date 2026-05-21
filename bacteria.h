#pragma once
#include <vector>
#include <cmath>
#include "raylib.h"
#include "raymath.h"

class Bacterium : public PhysicsBody
{
    float time  = 0.0f;
    float freq  = 3.0f;
    float amp   = 0.35f;
    float drag  = 0.97f;

    static const int BODY_NODES = 4;
    static const int FLAG_NODES = 14; 
    static const int TOTAL_NODES = BODY_NODES + FLAG_NODES;

public:
    Bacterium(float stiffness = 60.0f, float damping = 2.0f)
    {
        float segLen = 0.18f;  //space btwn flag nodes

        //body nodes (stiff pill along Z)
        nodes.push_back(Node({ 0.0f, 2.0f, -0.4f  }));  //0 --> front of body
        nodes.push_back(Node({ 0.0f, 2.0f, -0.13f }));  //1
        nodes.push_back(Node({ 0.0f, 2.0f,  0.13f }));  //2
        nodes.push_back(Node({ 0.0f, 2.0f,  0.4f  }));  //3 --> back of body

        //flaglellum nodes
        for (int i = 0; i < FLAG_NODES; i++)
        {
            float z = 0.4f + (i + 1) * segLen;
            nodes.push_back(Node({ 0.0f, 2.0f, z }));
        }

        float k = stiffness;
        float d = damping;

        //rigid body springs (rigid)
        addSpring(0,1, k, d);
        addSpring(1,2, k,d);
        addSpring(2,3, k,d);
        addSpring(0,2, k*0.5f, d);
        addSpring(1,3, k*0.5f, d);
        addSpring(0,3, k*0.3f, d);

        //flag springs (soft muscle)
        float fk = k * 0.02f, fd = d * 0.05f;
        for (int i = 0; i < FLAG_NODES; i++) {
            addSpring(BODY_NODES - 1 + i, BODY_NODES + i,fk,fd);
        }
    }

    // void animateFlagellum(float dt)
    // {
    //     time += dt;
    //     float phaseStep = 0.55f;
    //     for (int i = 0; i < FLAG_NODES; i++)
    //     {
    //         float phase = i * phaseStep;
    //         float force = 1.5f * std::sin(freq * time - phase);
    //         nodes[BODY_NODES + i].force.x += force;
    //     }
    // }
    void animateFlagellum(float dt)
    {
        time += dt;

        float phaseStep = 0.55f;
        for (int i = 0; i < FLAG_NODES; i++)
        {
            //t = 0 at body junction, 1 at tip
            //force tapers from 0 at base to full at tip
            float t = (float)i / (float)(FLAG_NODES - 1);
            float force = 1.5f * t * std::sin(freq * time - i * phaseStep);
            nodes[BODY_NODES + i].force.x += force;
        }
    }

    void updatePhysicsWithDrag(float dt)
    {

        for (auto& s : getSprings())
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f) {
                continue;
            }
            Vector3 dhat = Vector3Normalize(d);
            float fs = s.stiffness * (L - s.rest_length);
            float vrel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), dhat);
            float fd_f = s.damping * vrel;
            Vector3 F = Vector3Scale(dhat, -(fs + fd_f));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);
        }

        for (auto& n : nodes)
        {
            n.acceleration = Vector3Scale(n.force, 1.0f / n.mass);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.velocity = Vector3Scale(n.velocity, drag);
            n.position = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        }
    }

    void update(float dt)
    {
        for (auto& n : nodes) n.force = Vector3Zero();
        animateFlagellum(dt);
        updatePhysicsWithDrag(dt);
    }

    void draw()
    {
        auto& ns = getNodes();

        //green pill body 
        float radius = 0.18f;
        Color bodyCol = { 80, 220, 120, 255 };
        for (int i = 0; i < BODY_NODES - 1; i++)
            DrawCapsule(ns[i].position, ns[i+1].position, radius, 6, 4, bodyCol);
        DrawSphere(ns[0].position, radius, bodyCol);
        DrawSphere(ns[BODY_NODES-1].position, radius, bodyCol);

        //blue flagella 
        for (int i = BODY_NODES - 1; i < TOTAL_NODES - 1; i++)
        {
            float t = (float)(i - (BODY_NODES - 1)) / (float)FLAG_NODES;
            float thickness = 0.055f * (1.0f - t * 0.75f);  //tapers to thin tip
            Color col = { 100, 210, 255, 255 };
            DrawCapsule(ns[i].position, ns[i+1].position, thickness, 4, 2, col);
        }
    }
};