#include <memory>

#include <raylib.h>
#include <raymath.h>

#include "src/FluidEnvironment.h"
#include "src/PhysicsBody.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/DragForce.h"
#include "src/FlagellumDrive.h"
#include "src/Fish.h"
#include "src/Bacteria.h"
#include "src/PetriDish.h"

static void setupAquaticForces(PhysicsBody &body, const FluidEnvironment &fluid)
{
    body.addForceGenerator(std::make_unique<GravityForce>(fluid.gravity));
    body.addForceGenerator(std::make_unique<BuoyancyForce>(&fluid));
}

int main()
{
    PetriDish dish;

    FluidEnvironment water;
    water.density = 1000.0f;
    water.surfaceY = dish.ceilY();
    water.gravity = {0.0f, -9.81f, 0.0f};

    Bacteria bacterium;
    setupAquaticForces(bacterium, water);
    bacterium.addForceGenerator(std::make_unique<DragForce>(1.8f));
    bacterium.addForceGenerator(std::make_unique<FlagellumDrive>(
        Bacteria::BODY_NODES, Bacteria::FLAG_NODES));

    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "Micro-Life 3D");
    Camera3D camera = {{12.0f, 10.0f, 12.0f},
                       {0.0f, dish.ceilY() * 0.5f, 0.0f},
                       {0.0f, 1.0f, 0.0f},
                       45.0f,
                       CAMERA_PERSPECTIVE};

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime();

        bacterium.updatePhysicsExplicit(dt);
        dish.applyBoundary(bacterium.getNodes());

        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255});
        BeginMode3D(camera);

        dish.draw();
        bacterium.draw();

        EndMode3D();

        DrawFPS(10, screenHeight - 24);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
