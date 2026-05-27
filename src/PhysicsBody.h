#ifndef MICRO3D_PHYSICS_BODY_H
#define MICRO3D_PHYSICS_BODY_H

#include <vector>
#include <memory>
#include <cmath>

#include <raylib.h>
#include <raymath.h>
#include <Eigen/Sparse>

#include "Node.h"
#include "ForceGenerator.h"

class PhysicsBody
{
protected:
    std::vector<Node> nodes;
    std::vector<Spring> springs;
    std::vector<std::unique_ptr<ForceGenerator>> forceGenerators;

    void addSpring(int a, int b, float k, float d)
    {
        Vector3 pa = nodes[a].position, pb = nodes[b].position;
        float dx = pb.x - pa.x, dy = pb.y - pa.y, dz = pb.z - pa.z;
        float rest = std::sqrt(dx * dx + dy * dy + dz * dz);
        springs.emplace_back(&nodes[a], &nodes[b], rest, k, d);
    }

    void accumulateExternalForces(float dt)
    {
        for (auto &n : nodes)
            n.force = Vector3Zero();
        for (auto &fg : forceGenerators)
            fg->apply(*this, dt);
    }

    void resolveFloorCollision(float floorY, float restitution = 0.4f)
    {
        for (auto &n : nodes)
        {
            if (n.position.y < floorY)
            {
                n.position.y = floorY;
                if (n.velocity.y < 0.0f)
                    n.velocity.y = -n.velocity.y * restitution;
            }
        }
    }

public:
    virtual ~PhysicsBody() = default;

    std::vector<Node> &getNodes() { return nodes; }
    std::vector<Spring> &getSprings() { return springs; }

    void addForceGenerator(std::unique_ptr<ForceGenerator> fg)
    {
        forceGenerators.push_back(std::move(fg));
    }

    void updatePhysicsImplicit(float dt)
    {
        int N = (int)nodes.size();

        Eigen::SparseMatrix<float> A(3 * N, 3 * N);
        Eigen::VectorXf rhs = Eigen::VectorXf::Zero(3 * N);
        std::vector<Eigen::Triplet<float>> triplets;

        for (int i = 0; i < N; i++)
            for (int k = 0; k < 3; k++)
                triplets.push_back({3 * i + k, 3 * i + k, (float)nodes[i].mass});

        accumulateExternalForces(dt);

        for (auto &s : springs)
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f)
                continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            Vector3 F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);

            Eigen::Vector3f dh(d_hat.x, d_hat.y, d_hat.z);
            Eigen::Matrix3f M_outer = dh * dh.transpose();
            Eigen::Matrix3f M_perp = Eigen::Matrix3f::Identity() - M_outer;

            Eigen::Matrix3f K_spring = -s.stiffness * M_outer - s.stiffness * (1.0f - s.rest_length / L) * M_perp;
            Eigen::Matrix3f D_spring = -s.damping * M_outer;

            int na = (int)(s.nodeA - &nodes[0]);
            int nb = (int)(s.nodeB - &nodes[0]);
            Eigen::Matrix3f block = dt * dt * K_spring + dt * D_spring;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    triplets.push_back({3 * na + i, 3 * na + j, -block(i, j)});
                    triplets.push_back({3 * nb + i, 3 * nb + j, -block(i, j)});
                    triplets.push_back({3 * na + i, 3 * nb + j, block(i, j)});
                    triplets.push_back({3 * nb + i, 3 * na + j, block(i, j)});
                }

            Eigen::Vector3f va(nodes[na].velocity.x, nodes[na].velocity.y, nodes[na].velocity.z);
            Eigen::Vector3f vb(nodes[nb].velocity.x, nodes[nb].velocity.y, nodes[nb].velocity.z);

            Eigen::Vector3f contrib_a = dt * dt * (K_spring * vb - K_spring * va);
            Eigen::Vector3f contrib_b = dt * dt * (K_spring * va - K_spring * vb);

            rhs.segment<3>(3 * na) += contrib_a;
            rhs.segment<3>(3 * nb) += contrib_b;
        }

        A.setFromTriplets(triplets.begin(), triplets.end());

        for (int i = 0; i < N; i++)
        {
            rhs(3 * i + 0) += dt * nodes[i].force.x;
            rhs(3 * i + 1) += dt * nodes[i].force.y;
            rhs(3 * i + 2) += dt * nodes[i].force.z;
        }

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
        solver.compute(A);
        Eigen::VectorXf dv = solver.solve(rhs);

        for (int i = 0; i < N; i++)
        {
            nodes[i].velocity.x += dv(3 * i + 0);
            nodes[i].velocity.y += dv(3 * i + 1);
            nodes[i].velocity.z += dv(3 * i + 2);
            nodes[i].position.x += dt * nodes[i].velocity.x;
            nodes[i].position.y += dt * nodes[i].velocity.y;
            nodes[i].position.z += dt * nodes[i].velocity.z;
        }

        resolveFloorCollision(-5.0f);
    }

    void updatePhysicsExplicit(float dt)
    {
        accumulateExternalForces(dt);

        for (auto &s : springs)
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f)
                continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            Vector3 F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);
        }

        for (auto &n : nodes)
        {
            n.acceleration = Vector3Scale(n.force, 1.0f / n.mass);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.position = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        }

        resolveFloorCollision(-5.0f);
    }
};

#endif
