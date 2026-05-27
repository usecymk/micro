#include <iostream>
#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>
#include <Eigen/Sparse>


const Vector3 GLOBAL_GRAVITY = {0.0f, -9.81f, 0.0f};

struct Node
{
    Vector3 position, velocity, force, acceleration;
    double mass;
    Node(Vector3 pos, float m = 1.0f) : position(pos), velocity(Vector3Zero()), acceleration(Vector3Zero()), mass(m) {}
};

struct Spring
{
    Node *nodeA, *nodeB;
    float rest_length;
    float stiffness;
    float damping;
    Spring(Node *A, Node *B, float rl = 1.0f, float s = 1.0f, float d = 1.0f) : nodeA(A), nodeB(B), rest_length(rl), stiffness(s), damping(d) {}
};

class PhysicsBody
{
protected:
    std::vector<Node> nodes;
    std::vector<Spring> springs;

    void addSpring(int a, int b, float k, float d)
    {
        Vector3 pa = nodes[a].position, pb = nodes[b].position;
        float dx = pb.x - pa.x, dy = pb.y - pa.y, dz = pb.z - pa.z;
        float rest = std::sqrt(dx * dx + dy * dy + dz * dz);
        springs.emplace_back(&nodes[a], &nodes[b], rest, k, d);
    }

public:
    std::vector<Node> &getNodes() { return nodes; };
    std::vector<Spring> &getSprings() { return springs; };

    void updatePhysicsImplicit(float dt)
    {
        int N = nodes.size();

        Eigen::SparseMatrix<float> A(3 * N, 3 * N);
        Eigen::VectorXf rhs = Eigen::VectorXf::Zero(3 * N);

        std::vector<Eigen::Triplet<float>> triplets;

        // mass on diagonal
        for (int i = 0; i < N; i++)
            for (int k = 0; k < 3; k++)
                triplets.push_back({3 * i + k, 3 * i + k, (float)nodes[i].mass});

        for (auto &n : getNodes())
            n.force = Vector3Zero();

        for (auto &s : getSprings())
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f)
                continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            auto F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);

            Eigen::Vector3f dh(d_hat.x, d_hat.y, d_hat.z);
            Eigen::Matrix3f M_outer = dh * dh.transpose();
            Eigen::Matrix3f M_perp = Eigen::Matrix3f::Identity() - M_outer;

            Eigen::Matrix3f K_spring = -s.stiffness * M_outer - s.stiffness * (1.0f - s.rest_length / L) * M_perp;
            Eigen::Matrix3f D_spring = -s.damping * M_outer;

            int na = s.nodeA - &nodes[0];
            int nb = s.nodeB - &nodes[0];
            Eigen::Matrix3f block = dt * dt * K_spring + dt * D_spring;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    triplets.push_back({3 * na + i, 3 * na + j, -block(i, j)}); // A diagonal
                    triplets.push_back({3 * nb + i, 3 * nb + j, -block(i, j)}); // B diagonal
                    triplets.push_back({3 * na + i, 3 * nb + j, block(i, j)});  // off-diagonal AB
                    triplets.push_back({3 * nb + i, 3 * na + j, block(i, j)});  // off-diagonal BA
                };

            Eigen::Vector3f va(nodes[na].velocity.x, nodes[na].velocity.y, nodes[na].velocity.z);
            Eigen::Vector3f vb(nodes[nb].velocity.x, nodes[nb].velocity.y, nodes[nb].velocity.z);

            Eigen::Vector3f contrib_a = dt * dt * (K_spring * vb - K_spring * va);
            Eigen::Vector3f contrib_b = dt * dt * (K_spring * va - K_spring * vb);

            rhs.segment<3>(3 * na) += contrib_a;
            rhs.segment<3>(3 * nb) += contrib_b;
        };

        for (auto &n : getNodes())
            n.force = Vector3Add(n.force, Vector3Scale(GLOBAL_GRAVITY, n.mass));

        A.setFromTriplets(triplets.begin(), triplets.end());

        for (int i = 0; i < N; i++)
        {
            rhs(3 * i + 0) += dt * nodes[i].force.x;
            rhs(3 * i + 1) += dt * nodes[i].force.y;
            rhs(3 * i + 2) += dt * nodes[i].force.z;
        };

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

        const float floorY = -5.0f;
        for (auto &n : getNodes())
        {
            if (n.position.y < floorY)
            {
                n.position.y = floorY;
                if (n.velocity.y < 0.0f)
                    n.velocity.y = -n.velocity.y * 0.4f;
            }
        }
    };
    void updatePhysicsExplicit(float dt)
    {
        for (auto &n : getNodes())
            n.force = Vector3Zero();

        for (auto &s : getSprings())
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f)
                continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            auto F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);
        };

        for (auto &n : getNodes())
        {
            n.force = Vector3Add(n.force, Vector3Scale(GLOBAL_GRAVITY, n.mass));
            n.acceleration = Vector3Scale(n.force, 1.0f / n.mass);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.position = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        };

        // Plane collision at y = -5.0f
        const float floorY = -5.0f;
        for (auto &n : getNodes())
        {
            if (n.position.y < floorY)
            {
                n.position.y = floorY; // push node back above floor
                if (n.velocity.y < 0.0f)
                    n.velocity.y = -n.velocity.y * 0.4f; // e.g. restitution = 0.4f
            }
        }
    };
};

class Fish : public PhysicsBody
{
public:
    Fish(float stiffness = 8.0f, float damping = 0.6f)
    {
        int segments = 6;
        float length = 3.0f;

        nodes.reserve(segments * 4 + 3);

        for (int i = 0; i < segments; i++)
        {
            float t = (float)i / (segments - 1);
            float x = t * length;

            float radius = 0.4f * std::sin(t * 3.1415f);

            float w = radius;
            float h = radius * 0.6f;

            nodes.push_back(Node({x, 0, -w}));
            nodes.push_back(Node({x, 0, w}));
            nodes.push_back(Node({x, h, 0}));
            nodes.push_back(Node({x, -h, 0}));
        }

        int tailBase = nodes.size();
        float tailX = length;

        nodes.push_back(Node({tailX + 0.5f, 0.0f, 0.0f}));
        nodes.push_back(Node({tailX + 0.8f, 0.4f, 0.0f}));
        nodes.push_back(Node({tailX + 0.8f, -0.4f, 0.0f}));

        for (int i = 0; i < segments - 1; i++)
        {
            int base = i * 4;
            int next = (i + 1) * 4;

            for (int j = 0; j < 4; j++)
                addSpring(base + j, next + j, stiffness, damping);

            addSpring(base + 0, next + 1, stiffness, damping);
            addSpring(base + 1, next + 0, stiffness, damping);
            addSpring(base + 2, next + 3, stiffness, damping);
            addSpring(base + 3, next + 2, stiffness, damping);
        }

        for (int i = 0; i < segments; i++)
        {
            int b = i * 4;

            addSpring(b + 0, b + 2, stiffness, damping);
            addSpring(b + 2, b + 1, stiffness, damping);
            addSpring(b + 1, b + 3, stiffness, damping);
            addSpring(b + 3, b + 0, stiffness, damping);

            addSpring(b + 0, b + 1, stiffness, damping);
            addSpring(b + 2, b + 3, stiffness, damping);
        }

        float tk = stiffness * 0.4f;

        int last = (segments - 1) * 4;

        addSpring(last + 0, tailBase, tk, damping);
        addSpring(last + 1, tailBase, tk, damping);
        addSpring(last + 2, tailBase, tk, damping);
        addSpring(last + 3, tailBase, tk, damping);

        addSpring(tailBase, tailBase + 1, tk, damping);
        addSpring(tailBase, tailBase + 2, tk, damping);
        addSpring(tailBase + 1, tailBase + 2, tk, damping);
    }
};

class Cube : public PhysicsBody
{
public:
    Cube(float size = 1.0f, float stiffness = 50.0f, float damping = 5.0f)
    {
        // 8 corner nodes of the cube
        nodes.reserve(8);
        float h = size / 2.0f;
        nodes.push_back(Node(Vector3{-h, -h, -h}));
        nodes.push_back(Node(Vector3{h, -h, -h}));
        nodes.push_back(Node(Vector3{-h, h, -h}));
        nodes.push_back(Node(Vector3{h, h, -h}));
        nodes.push_back(Node(Vector3{-h, -h, h}));
        nodes.push_back(Node(Vector3{h, -h, h}));
        nodes.push_back(Node(Vector3{-h, h, h}));
        nodes.push_back(Node(Vector3{h, h, h}));

        // 12 edges
        addSpring(0, 1, stiffness, damping);
        addSpring(2, 3, stiffness, damping);
        addSpring(4, 5, stiffness, damping);
        addSpring(6, 7, stiffness, damping); // along X
        addSpring(0, 2, stiffness, damping);
        addSpring(1, 3, stiffness, damping);
        addSpring(4, 6, stiffness, damping);
        addSpring(5, 7, stiffness, damping); // along Y
        addSpring(0, 4, stiffness, damping);
        addSpring(1, 5, stiffness, damping);
        addSpring(2, 6, stiffness, damping);
        addSpring(3, 7, stiffness, damping); // along Z

        // 12 face diagonals (2 per face)
        addSpring(0, 3, stiffness, damping);
        addSpring(1, 2, stiffness, damping); // bottom face
        addSpring(4, 7, stiffness, damping);
        addSpring(5, 6, stiffness, damping); // top face
        addSpring(0, 5, stiffness, damping);
        addSpring(1, 4, stiffness, damping); // front face
        addSpring(2, 7, stiffness, damping);
        addSpring(3, 6, stiffness, damping); // back face
        addSpring(0, 6, stiffness, damping);
        addSpring(2, 4, stiffness, damping); // left face
        addSpring(1, 7, stiffness, damping);
        addSpring(3, 5, stiffness, damping); // right face

        // 4 body diagonals
        addSpring(0, 7, stiffness, damping);
        addSpring(1, 6, stiffness, damping);
        addSpring(2, 5, stiffness, damping);
        addSpring(3, 4, stiffness, damping);
    };
};

#include "amoeba.hpp"
#include "cocci.hpp"

int main()
{
    //Cube cube;
    Fish fish;
    //Amoeba amoeba({0.0f, -2.0f, 0.0f});
    CocciCluster myStaph({-3.0f, -1.0f, 0.0f});
    //Bait myBait({4.0f, -4.5f, 4.0f});
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "Fish Sim");
    Camera3D camera = {{5, 5, 5}, {0, 0, 0}, {0, 1, 0}, 45.0f, CAMERA_PERSPECTIVE};

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        Vector2 mouseDelta = GetMouseDelta();
        UpdateCamera(&camera, CAMERA_FREE);
        //cube.updatePhysicsImplicit(GetFrameTime());

        float dt = GetFrameTime();

        

        myStaph.actuate(dt);
        //run biological motor and friction
        //amoeba.actuate(GetFrameTime(), preyLocation); 

        Vector3 preyLocation = myStaph.getCenterPosition();

        Vector3 diff = Vector3Subtract(preyLocation, amoebaLocation);
        diff.y = 0.0f;
        float distance = Vector3Length(diff);

        // If distance is less than the combined radii (~1.8f), the prey is eaten
        if (distance < 1.8f) 
        {
            // Respawn the cluster 15 units away from the amoeba
            myStaph.Respawn(amoebaLocation, 15.0f);
        }

        myStaph.updatePhysicsImplicit(dt);
        
        // run mass-spring-damper physics solver
        //amoeba.updatePhysicsImplicit(dt);

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
        DrawPlane((Vector3){0.0f, -5.0f, 0.0f}, (Vector2){32.0f, 32.0f}, ORANGE);
        // for (auto &n : cube.getNodes())
        //     DrawSphere(n.position, 0.05f, GREEN);
        // for (auto &s : cube.getSprings())
        //     DrawLine3D(s.nodeA->position, s.nodeB->position, WHITE);

        //for (size_t i = 1; i < amoeba.getNodes().size(); i++)
            //DrawSphere(amoeba.getNodes()[i].position, 0.1f, PURPLE);
            
        // draw the nucleus
        //DrawSphere(amoeba.getNodes()[0].position, 0.25f, RED);

        //myBait.Draw();

        //draw the springs
        //for (auto &s : amoeba.getSprings())
            //DrawLine3D(s.nodeA->position, s.nodeB->position, Fade(WHITE, 0.3f));

        //drawing the coccus
        for (auto& cell : myStaph.getCells())
        {
            for (int i = cell.startIndex; i <= cell.endIndex; i++)
            {
                DrawSphere(myStaph.getNodes()[i].position, 0.08f, SKYBLUE);
            }
            DrawSphere(myStaph.getNodes()[cell.centerIndex].position, 0.15f, BLUE);
        }
        for (auto &s : myStaph.getSprings())
        {
            DrawLine3D(s.nodeA->position, s.nodeB->position, Fade(SKYBLUE, 0.2f));
        }

        EndMode3D();
        EndDrawing();
    };

    return 0;
};
