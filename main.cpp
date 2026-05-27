#include <memory>

#include <raylib.h>
#include <raymath.h>

#include "src/FluidEnvironment.h"
#include "src/PhysicsBody.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/Cube.h"
#include "src/Fish.h"

static void setupAquaticForces(PhysicsBody &body, const FluidEnvironment &fluid)
{
    body.addForceGenerator(std::make_unique<GravityForce>(fluid.gravity));
    body.addForceGenerator(std::make_unique<BuoyancyForce>(&fluid));
}

int main()
{
    FluidEnvironment water;
    water.density = 1000.0f;
    water.surfaceY = 2.0f;
    water.gravity = {0.0f, -9.81f, 0.0f};

    Cube cube;
    setupAquaticForces(cube, water);

    Fish fish;
    setupAquaticForces(fish, water);

    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "Micro-Life 3D");
    Camera3D camera = {{8, 6, 8}, {0, 0, 0}, {0, 1, 0}, 45.0f, CAMERA_PERSPECTIVE};

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime();
        cube.updatePhysicsImplicit(dt);

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);

        DrawPlane((Vector3){0.0f, -5.0f, 0.0f}, (Vector2){32.0f, 32.0f}, ORANGE);

        Color waterColor = (Color){80, 140, 220, 90};
        DrawPlane((Vector3){0.0f, water.surfaceY, 0.0f}, (Vector2){32.0f, 32.0f}, waterColor);

        for (auto &n : cube.getNodes())
            DrawSphere(n.position, 0.05f, GREEN);
        for (auto &s : cube.getSprings())
            DrawLine3D(s.nodeA->position, s.nodeB->position, WHITE);

        EndMode3D();

        DrawFPS(10, screenHeight - 24);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
