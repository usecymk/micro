#include <vector>
#include <memory>
#include <cmath>

#include "raylib.h"
#include "raymath.h"

#include "src/PhysicsBody.h"
#include "src/FluidEnvironment.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/Bacteria.h"
#include "src/PetriDish.h"
#include "src/BoidBehavior.h"

static BoidState snapshotState(Bacteria &b)
{
    auto &ns = b.getNodes();
    Vector3 pos = Vector3Zero(), vel = Vector3Zero();
    for (int i = 0; i < Bacteria::BODY_NODES; i++)
    {
        pos = Vector3Add(pos, ns[i].position);
        vel = Vector3Add(vel, ns[i].velocity);
    }
    float inv = 1.0f / Bacteria::BODY_NODES;
    return {Vector3Scale(pos, inv), Vector3Scale(vel, inv)};
}

int main()
{
    PetriDish dish;

    FluidEnvironment water;
    water.density  = 1000.0f;
    water.surfaceY = dish.ceilY();
    water.gravity  = {0.0f, -9.81f, 0.0f};

    const int N = 10;

    std::vector<BoidState> flockStates(N);

    BoidBehavior boidParams;
    boidParams.separationRadius = 0.18f;
    boidParams.alignmentRadius  = 0.8f;
    boidParams.cohesionRadius   = 0.8f;
    boidParams.separationWeight = 1.8f;
    boidParams.alignmentWeight  = 2.5f;
    boidParams.cohesionWeight   = 1.5f;
    boidParams.maxForce         = 0.15f;

    std::vector<Bacteria> bacteria(N);
    for (int i = 0; i < N; i++)
    {
        float angle = (float)i / N * 2.0f * PI;
        float r     = 0.12f + 0.08f * ((float)(i % 3) / 2.0f);
        Vector3 off = {r * cosf(angle), 0.0f, r * sinf(angle)};

        for (auto &n : bacteria[i].getNodes())
            n.position = Vector3Add(n.position, off);

        //external forces only; FlagellumDrive and DragForce == internal
        bacteria[i].addForceGenerator(std::make_unique<GravityForce>(water.gravity));
        bacteria[i].addForceGenerator(std::make_unique<BuoyancyForce>(&water));
        bacteria[i].addForceGenerator(
            std::make_unique<BoidForceGenerator>(&flockStates, i, Bacteria::BODY_NODES, &boidParams));
    }

    InitWindow(1024, 600, "Boid Bacteria Test");

    Camera3D camera = {
        {0.8f, 2.8f, 0.8f},
        {0.0f, 2.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        45.0f,
        CAMERA_PERSPECTIVE};

    bool showDebug = false;

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime();

        if (IsKeyDown(KEY_T))
        {
            boidParams.cohesionWeight   = 3.5f;
            boidParams.separationWeight = 0.8f;
            boidParams.alignmentWeight  = 2.5f;
        }
        else if (IsKeyDown(KEY_X))
        {
            boidParams.cohesionWeight   = 0.3f;
            boidParams.separationWeight = 4.0f;
            boidParams.alignmentWeight  = 0.5f;
        }
        else if (IsKeyDown(KEY_R))
        {
            boidParams.cohesionWeight   = 1.5f;
            boidParams.separationWeight = 1.8f;
            boidParams.alignmentWeight  = 2.5f;
        }
        if (IsKeyPressed(KEY_G)) showDebug = !showDebug;

        for (int i = 0; i < N; i++)
            flockStates[i] = snapshotState(bacteria[i]);

        for (int i = 0; i < N; i++)
        {
            bacteria[i].update(dt);
            dish.applyBoundary(bacteria[i].getNodes());

            //trigger escape behavior when near the dish wall
            Vector3 com = bacteria[i].getCenterOfMass();
            float r = sqrtf(com.x * com.x + com.z * com.z);
            if (r > dish.radius * 0.82f)
            {
                Vector3 awayDir = {-com.x / r, 0.0f, -com.z / r};
                bacteria[i].onWallHit(awayDir);
            }
        }

        BeginDrawing();
        ClearBackground({5, 10, 20, 255});
        BeginMode3D(camera);

        dish.draw();
        for (auto &b : bacteria)
            b.draw(showDebug);

        EndMode3D();

        if (showDebug)
        {
            for (int i = 0; i < N; i++)
            {
                if (!bacteria[i].bsm.state.alive) continue;

                Vector3 com = bacteria[i].getCenterOfMass();
                Vector2 sp  = GetWorldToScreen(com, camera);

                int x = (int)sp.x - 30;
                int y = (int)sp.y - 36;
                int w = 60, h = 5;

                //hunger bar (orange)
                DrawRectangle(x, y,     w, h, {40, 40, 40, 180});
                DrawRectangle(x, y,     (int)(w * bacteria[i].bsm.state.hunger), h, ORANGE);
                DrawRectangleLines(x, y, w, h, GRAY);

                //fear bar (red)
                DrawRectangle(x, y + 7, w, h, {40, 40, 40, 180});
                DrawRectangle(x, y + 7, (int)(w * bacteria[i].bsm.state.fear),   h, RED);
                DrawRectangleLines(x, y + 7, w, h, GRAY);
            }
        }

        DrawFPS(10, 576);
        DrawText("T=tighten  X=disperse  R=reset  G=debug  |  WASD+mouse=camera", 10, 10, 16, RAYWHITE);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
