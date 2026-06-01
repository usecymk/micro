// #include <me mory>

#include <vector>
#include <memory>
#include <cmath>
#include <raylib.h>
#include <raymath.h>

#include "src/FluidEnvironment.h"
#include "src/PhysicsBody.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/DragForce.h"
#include "src/Bacteria.h"
#include "src/Amoeba.h"
#include "src/Cocci.h"
#include "src/Bait.h"
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

    Amoeba amoeba({0.0f, dish.floorY + 2.5f, 0.0f});
    amoeba.setFloorY(dish.floorY);
    amoeba.addForceGenerator(std::make_unique<DragForce>(0.6f));


    CocciCluster cocci({-3.0f, dish.floorY + 2.0f, 0.0f});
    cocci.addForceGenerator(std::make_unique<DragForce>(0.4f));


    //Bait bait({4.0f, dish.floorY + 0.5f, 4.0f});

    // ── boid bacteria flock ───────────────────────────────────────────────────
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

    std::vector<Bacteria> flock(N);
    for (int i = 0; i < N; i++)
    {
        float angle = (float)i / N * 2.0f * PI;
        float r     = 0.12f + 0.08f * ((float)(i % 3) / 2.0f);
        Vector3 off = {r * cosf(angle), 0.0f, r * sinf(angle)};

        for (auto &n : flock[i].getNodes())
            n.position = Vector3Add(n.position, off);

        flock[i].addForceGenerator(std::make_unique<GravityForce>(water.gravity));
        flock[i].addForceGenerator(std::make_unique<BuoyancyForce>(&water));
        flock[i].addForceGenerator(
            std::make_unique<BoidForceGenerator>(&flockStates, i, Bacteria::BODY_NODES, &boidParams));
    }
    // ─────────────────────────────────────────────────────────────────────────

    const int screenWidth  = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "Micro-Life 3D");
    Camera3D camera = {{12.0f, 10.0f, 12.0f},
                       {0.0f, dish.ceilY() * 0.5f, 0.0f},
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

        Vector3 hunter = amoeba.getCenterPosition();
        Vector3 prey   = cocci.getCenterPosition();

        Vector3 dxCocci = Vector3Subtract(prey,          hunter); dxCocci.y = 0.0f;
        //Vector3 dxBait  = Vector3Subtract(bait.position, hunter); dxBait.y  = 0.0f;

        float distCocci = Vector3Length(dxCocci);
        //float distBait  = Vector3Length(dxBait);
        Vector3 target  = prey;

        // ── boid flock update ─────────────────────────────────────────────────
        if (IsKeyDown(KEY_T)) {
            boidParams.cohesionWeight   = 3.5f;
            boidParams.separationWeight = 0.8f;
            boidParams.alignmentWeight  = 2.5f;
        } else if (IsKeyDown(KEY_X)) {
            boidParams.cohesionWeight   = 0.3f;
            boidParams.separationWeight = 4.0f;
            boidParams.alignmentWeight  = 0.5f;
        } else if (IsKeyDown(KEY_R)) {
            boidParams.cohesionWeight   = 1.5f;
            boidParams.separationWeight = 1.8f;
            boidParams.alignmentWeight  = 2.5f;
        }

        if (IsKeyPressed(KEY_G)) showDebug = !showDebug;

        for (int i = 0; i < N; i++)
            flockStates[i] = snapshotState(flock[i]);

        amoeba.actuate(dt, cocci, flockStates);
        cocci.actuate(dt);

        if (distCocci < 1.8f)
            cocci.respawn(hunter, dish.radius * 0.85f, dish.floorY + 2.0f);
        // if (distBait < 1.2f)
        //     bait.respawn(hunter, dish.radius * 0.85f, dish.floorY);

        amoeba.updatePhysicsImplicit(dt);
        cocci.updatePhysicsImplicit(dt);
        dish.applyBoundary(amoeba.getNodes());
        dish.applyBoundary(cocci.getNodes());

        
        

        for (int i = 0; i < N; i++)
        {
            flock[i].update(dt);
            dish.applyBoundary(flock[i].getNodes());

            Vector3 com = flock[i].getCenterOfMass();
            float r = sqrtf(com.x * com.x + com.z * com.z);
            if (r > dish.radius * 0.82f)
            {
                Vector3 awayDir = {-com.x / r, 0.0f, -com.z / r};
                flock[i].onWallHit(awayDir);
            }
        }
        // ─────────────────────────────────────────────────────────────────────

        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255});
        BeginMode3D(camera);

        dish.draw();
        amoeba.draw();
        cocci.draw();
        //bait.draw();
        for (auto &b : flock)
            b.draw(showDebug);

        dish.drawShell();

        EndMode3D();

        float camTemp = dish.temperatureAt(camera.position);
        DrawText(TextFormat("Camera: %.1f C", camTemp), 10, 10, 20, RAYWHITE);
        if (showDebug)
        {
            for (int i = 0; i < N; i++)
            {
                if (!flock[i].bsm.state.alive) continue;

                Vector3 com = flock[i].getCenterOfMass();
                Vector2 sp  = GetWorldToScreen(com, camera);

                int x = (int)sp.x - 30;
                int y = (int)sp.y - 36;
                int w = 60, h = 5;

                DrawRectangle(x, y,     w, h, {40, 40, 40, 180});
                DrawRectangle(x, y,     (int)(w * flock[i].bsm.state.hunger), h, ORANGE);
                DrawRectangleLines(x, y, w, h, GRAY);

                DrawRectangle(x, y + 7, w, h, {40, 40, 40, 180});
                DrawRectangle(x, y + 7, (int)(w * flock[i].bsm.state.fear),   h, RED);
                DrawRectangleLines(x, y + 7, w, h, GRAY);
            }
        }

        DrawFPS(10, screenHeight - 24);
        DrawText("T=tighten  X=disperse  R=reset  G=debug  |  WASD+mouse=camera", 10, 10, 16, RAYWHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
