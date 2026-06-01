#include <vector>
#include <memory>
#include <cmath>
#include <raylib.h>
#include <raymath.h>

#include "src/FluidEnvironment.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/DragForce.h"
#include "src/Bacteria.h"
#include "src/Amoeba.h"
#include "src/Cocci.h"
#include "src/PetriDish.h"
#include "src/BoidBehavior.h"
#include "src/SimulationUtils.h"

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

    std::vector<std::unique_ptr<Amoeba>> amoebas;
    auto addAmoeba = [&](Vector3 spawn)
    {
        amoebas.push_back(std::make_unique<Amoeba>(spawn));
        amoebas.back()->setFloorY(dish.floorY);
        amoebas.back()->addForceGenerator(std::make_unique<DragForce>(0.6f));
    };

    addAmoeba({0.0f, dish.floorY + 2.5f, 0.0f});
    addAmoeba({-5.0f, dish.floorY + 1.5f, -4.0f});

    CocciCluster cocci({-3.0f, dish.floorY + 2.0f, 0.0f});
    cocci.addForceGenerator(std::make_unique<DragForce>(0.4f));

    // ── boid bacteria flock ───────────────────────────────────────────────────
    const int N = 100;
    std::vector<BoidState> flockStates(N);

    BoidBehavior boidParams;
    boidParams.separationRadius = 0.34f;
    boidParams.alignmentRadius  = 0.8f;
    boidParams.cohesionRadius   = 1.0f;
    boidParams.separationWeight = 1.05f;
    boidParams.alignmentWeight  = 1.6f;
    boidParams.cohesionWeight   = 0.75f;
    boidParams.maxForce         = 0.07f;

    std::vector<Bacteria> flock(N);
    for (int i = 0; i < N; i++)
    {
        float t = ((float)i + 0.5f) / (float)N;
        float angle = (float)i * 2.399963f; // golden angle for even disk fill
        float innerRadius = 2.2f;
        float outerRadius = dish.radius * 0.72f;
        float r = innerRadius + (outerRadius - innerRadius) * std::sqrt(t);
        Vector3 spawn = {r * cosf(angle), dish.floorY + 2.0f, r * sinf(angle)};
        flock[i].reset(spawn);
        flock[i].bsm.setRandomSeed(0x9E3779B9u * (unsigned int)(i + 1));

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
    const float simulationSpeed = 3.0f;
    std::vector<Vector3> predatorPositions;

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime() * simulationSpeed;

        // ── boid flock update ─────────────────────────────────────────────────
        if (IsKeyDown(KEY_T)) {
            boidParams.cohesionWeight   = 1.4f;
            boidParams.separationWeight = 0.7f;
            boidParams.alignmentWeight  = 1.6f;
        } else if (IsKeyDown(KEY_X)) {
            boidParams.cohesionWeight   = 0.2f;
            boidParams.separationWeight = 2.0f;
            boidParams.alignmentWeight  = 0.35f;
        } else if (IsKeyDown(KEY_R)) {
            boidParams.cohesionWeight   = 0.75f;
            boidParams.separationWeight = 1.05f;
            boidParams.alignmentWeight  = 1.6f;
        }

        if (IsKeyPressed(KEY_G)) showDebug = !showDebug;

        predatorPositions.clear();
        for (const auto &amoeba : amoebas)
            predatorPositions.push_back(amoeba->getCenterPosition());
        boidParams.setPredatorPositions(predatorPositions);

        for (int i = 0; i < N; i++)
            flockStates[i] = snapshotState(flock[i]);

        for (auto &amoeba : amoebas)
        {
            Vector3 amoebaPos = amoeba->getCenterPosition();
            amoeba->actuate(
                dt,
                flockStates,
                dish.radius,
                &cocci,
                dish.temperatureAt(amoebaPos),
                dish.temperatureGradientAt(amoebaPos));
        }
        cocci.actuate(dt);

        for (auto &amoeba : amoebas)
        {
            amoeba->updatePhysicsImplicit(dt);
            dish.applyBoundary(amoeba->getNodes());
        }
        cocci.updatePhysicsImplicit(dt);
        dish.applyBoundary(cocci.getNodes());

        predatorPositions.clear();
        for (const auto &amoeba : amoebas)
            predatorPositions.push_back(amoeba->getCenterPosition());
        boidParams.setPredatorPositions(predatorPositions);

        for (int i = 0; i < N; i++)
        {
            Vector3 bacteriaPos = flockStates[i].position;
            flock[i].bsm.updateTemperaturePreference(
                dish.temperatureAt(bacteriaPos),
                dish.temperatureGradientAt(bacteriaPos));
            flock[i].bsm.updateVerticalBoundsAvoidance(
                bacteriaPos.y,
                dish.floorY + 0.18f,
                dish.ceilY() - 0.18f);

            float preUpdateRadius = sqrtf(bacteriaPos.x * bacteriaPos.x + bacteriaPos.z * bacteriaPos.z);
            if (preUpdateRadius > dish.radius * 0.82f)
            {
                Vector3 awayDir = {-bacteriaPos.x / preUpdateRadius, 0.0f, -bacteriaPos.z / preUpdateRadius};
                flock[i].onWallHit(awayDir, 0.35f);
            }

            flock[i].bsm.updatePredatorThreat(
                bacteriaPos,
                closestPredatorPosition(bacteriaPos, predatorPositions));
            flock[i].update(dt);
            dish.applyBoundary(flock[i].getNodes());
        }
        consumeCapturedBacteria(amoebas, flock, cocci, dish);
        resolveAmoebaBacteriaSoftCollisions(amoebas, flock, dish);
        resolveAmoebaSoftCollisions(amoebas, dish);
        resolveBacteriaSoftCollisions(flock, dish);
        // ─────────────────────────────────────────────────────────────────────

        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255});
        BeginMode3D(camera);

        dish.draw();
        for (auto &amoeba : amoebas)
            amoeba->draw();
        cocci.draw();
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
