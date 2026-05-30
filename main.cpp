#include <memory>

#include <raylib.h>
#include <raymath.h>

#include "src/FluidEnvironment.h"
#include "src/PhysicsBody.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/DragForce.h"
#include "src/FlagellumDrive.h"
#include "src/Bacteria.h"
#include "src/Amoeba.h"
#include "src/Cocci.h"
#include "src/Bait.h"
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
    water.density  = 1000.0f;
    water.surfaceY = dish.ceilY();
    water.gravity  = {0.0f, -9.81f, 0.0f};

    Bacteria bacterium;
    setupAquaticForces(bacterium, water);
    bacterium.addForceGenerator(std::make_unique<DragForce>(1.8f));
    bacterium.addForceGenerator(std::make_unique<FlagellumDrive>(
        Bacteria::BODY_NODES, Bacteria::FLAG_NODES));


    Amoeba amoeba({0.0f, dish.floorY + 2.5f, 0.0f});
    amoeba.setFloorY(dish.floorY);
    amoeba.addForceGenerator(std::make_unique<DragForce>(0.6f));


    CocciCluster cocci({-3.0f, dish.floorY + 2.0f, 0.0f});
    cocci.addForceGenerator(std::make_unique<DragForce>(0.4f));


    Bait bait({4.0f, dish.floorY + 0.5f, 4.0f});

    const int screenWidth  = 800;
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

        Vector3 hunter = amoeba.getCenterPosition();
        Vector3 prey   = cocci.getCenterPosition();

        Vector3 dxCocci = Vector3Subtract(prey,          hunter); dxCocci.y = 0.0f;
        Vector3 dxBait  = Vector3Subtract(bait.position, hunter); dxBait.y  = 0.0f;

        float distCocci = Vector3Length(dxCocci);
        float distBait  = Vector3Length(dxBait);
        Vector3 target  = (distBait < distCocci) ? bait.position : prey;

        amoeba.actuate(dt, target);
        cocci.actuate(dt);

        if (distCocci < 1.8f)
            cocci.respawn(hunter, dish.radius * 0.85f, dish.floorY + 2.0f);
        if (distBait < 1.2f)
            bait.respawn(hunter, dish.radius * 0.85f, dish.floorY);

        amoeba.updatePhysicsImplicit(dt);
        cocci.updatePhysicsImplicit(dt);
        dish.applyBoundary(amoeba.getNodes());
        dish.applyBoundary(cocci.getNodes());

        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255});
        BeginMode3D(camera);

        dish.draw();
        bacterium.draw();
        amoeba.draw();
        cocci.draw();
        bait.draw();

        EndMode3D();

        DrawFPS(10, screenHeight - 24);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
