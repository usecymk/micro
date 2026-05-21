#include <vector>
#include <cmath>
#include "raylib.h"
#include "raymath.h"

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
        float rest = std::sqrt(dx*dx + dy*dy + dz*dz);
        springs.emplace_back(&nodes[a], &nodes[b], rest, k, d);
    }

public:
    std::vector<Node> &getNodes() { return nodes; }
    std::vector<Spring> &getSprings() { return springs; }

    void updatePhysicsExplicit(float dt)
    {
        for (auto &n : nodes) n.force = Vector3Zero();

        for (auto &s : springs)
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f) continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            auto F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);
        }

        for (auto &n : nodes)
        {
            n.force = Vector3Add(n.force, Vector3Scale(GLOBAL_GRAVITY, n.mass));
            n.acceleration = Vector3Scale(n.force, 1.0f / n.mass);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.position = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        }
    }
};

#include "bacteria.h"
#include "petri.h"

int main()
{
    InitWindow(800, 450, "Bacterium Test");

    Camera3D camera = {};
    camera.position = {  6.0f, 4.0f, 0.5f }; 
    camera.target = {  0.0f, 4.0f, 0.5f }; 
    camera.up = {  0.0f,  1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();
    SetTargetFPS(60);

    Bacterium bact;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        bact.update(dt);
        ApplyDishBoundaryAll(bact.getNodes());

        BeginDrawing();
        ClearBackground({ 5, 10, 20, 255 });

        BeginMode3D(camera);
            UpdateCamera(&camera, CAMERA_FREE);
            DrawPetriDish();
            bact.draw();
        EndMode3D();

        DrawFPS(10, 10);
        DrawText("WASD + mouse to move camera", 10, 30, 16, GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}