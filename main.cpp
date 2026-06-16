// #include <me mory>

#include <vector>
#include <memory>
#include <cmath>
#include <cstdio>
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
#include "src/Nutrient.h"
// #include "src/Spirogyra.h"
#include "src/PetriDish.h"
#include "src/BoidBehavior.h"

static constexpr int BOID_MAX = 32;
static constexpr int BOID_INIT = 16;
static constexpr int NUM_GROUPS = 3;
static constexpr float DISPERSE_DUR = 5.0f;
static constexpr float AMOEBA_TEMP_TARGET = 40.0f;
static constexpr float PREDATOR_AVOID_RADIUS = 2.2f;
static constexpr float BACTERIA_CONSUME_RADIUS = 0.75f;
static constexpr float BACTERIA_NUTRITION = 18.0f;
static constexpr float NUTRIENT_FEED_THRESHOLD = 0.18f;
static constexpr float NUTRIENT_BITE = 0.9f;
static constexpr float NUTRIENT_FEED_RATE = 0.8f;

struct BoidGroup
{
    std::vector<BoidState> flockStates;
    std::vector<std::unique_ptr<Bacteria>> members;
    std::vector<float> hitCooldown;
    BoidBehavior params;
    float disperseTimer = 0.0f;
    float spawnTimer = 1.0f;
    Vector3 spawnCenter = {0.0f, 2.0f, 0.0f};
    int spawnCount = 0;

    int liveCount() const
    {
        int n = 0;
        for (auto &b : members)
            n += b->bsm.state.alive ? 1 : 0;
        return n;
    }
    int deadSlot() const
    {
        for (int i = 0; i < BOID_MAX; i++)
            if (!members[i]->bsm.state.alive)
                return i;
        return -1;
    }
};

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

static bool isLookingAt(Vector3 worldPos, const Camera3D &camera, int screenWidth, int screenHeight)
{
    Vector3 camDir = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    Vector3 toTarget = Vector3Subtract(worldPos, camera.position);
    if (Vector3DotProduct(camDir, toTarget) <= 0.0f)
        return false;

    Vector2 screenPos = GetWorldToScreen(worldPos, camera);
    if (screenPos.x < 0.0f || screenPos.x > screenWidth || screenPos.y < 0.0f || screenPos.y > screenHeight)
        return false;

    float dx = screenPos.x - screenWidth * 0.5f;
    float dy = screenPos.y - screenHeight * 0.5f;
    return std::sqrt(dx * dx + dy * dy) < 95.0f;
}

static void drawStatusBar(int x, int y, int w, int h, float value01, float threshold01, Color fill, const char *label)
{
    value01 = Clamp(value01, 0.0f, 1.0f);
    threshold01 = Clamp(threshold01, 0.0f, 1.0f);

    DrawText(label, x, y - 15, 12, {215, 230, 240, 255});
    DrawRectangle(x, y, w, h, {18, 28, 38, 220});
    DrawRectangle(x, y, (int)(w * value01), h, fill);
    DrawRectangleLines(x, y, w, h, {120, 150, 170, 220});

    int tx = x + (int)(w * threshold01);
    DrawLine(tx, y - 3, tx, y + h + 3, RAYWHITE);
}

static void drawAmoebaStatusPanel(const Amoeba &amoeba, float currentTemp, int screenWidth)
{
    const int x = screenWidth - 285;
    const int y = 72;
    const int w = 250;
    const int barW = 210;
    const int barH = 10;

    DrawRectangleRounded({(float)x, (float)y, (float)w, 142.0f}, 0.10f, 8, {6, 14, 24, 210});
    DrawRectangleRoundedLines({(float)x, (float)y, (float)w, 142.0f}, 0.10f, 8, {95, 170, 210, 180});

    DrawText("Amoeba", x + 14, y + 10, 18, RAYWHITE);
    DrawText(TextFormat("State: %s", amoeba.getStateName()), x + 14, y + 32, 13, {215, 230, 240, 255});

    drawStatusBar(x + 14, y + 62, barW, barH,
                  amoeba.getHunger() / 100.0f,
                  amoeba.getHungerThreshold() / 100.0f,
                  ORANGE,
                  TextFormat("Hunger %.0f / 100", amoeba.getHunger()));

    float tempMin = 18.0f;
    float tempMax = 50.0f;
    drawStatusBar(x + 14, y + 91, barW, barH,
                  (currentTemp - tempMin) / (tempMax - tempMin),
                  (AMOEBA_TEMP_TARGET - tempMin) / (tempMax - tempMin),
                  {80, 190, 255, 255},
                  TextFormat("Temperature %.1f C", currentTemp));

    drawStatusBar(x + 14, y + 120, barW, barH,
                  amoeba.getTemperatureStress() / 100.0f,
                  amoeba.getTemperatureThreshold() / 100.0f,
                  {180, 120, 255, 255},
                  TextFormat("Temp stress %.0f / 100", amoeba.getTemperatureStress()));
}

int main()
{
    PetriDish dish;

    FluidEnvironment water;
    water.density = 1000.0f;
    water.surfaceY = dish.ceilY();
    water.gravity = {0.0f, -9.81f, 0.0f};

    Amoeba amoeba({0.0f, dish.floorY + 2.5f, 0.0f});
    amoeba.setFloorY(dish.floorY);
    amoeba.addForceGenerator(std::make_unique<DragForce>(0.6f));

    CocciCluster cocci({-3.0f, dish.floorY + 2.0f, 0.0f});
    cocci.addForceGenerator(std::make_unique<DragForce>(0.4f));

    // ── 3 boid groups, 16 flagella each ──────────────────────────────────────
    BoidGroup groups[NUM_GROUPS];

    NutrientField nutrientField;
    nutrientField.init(dish.radius, dish.floorY, dish.ceilY(), 1400, 7);

    // Spirogyra spirogyra({-2.0f, dish.floorY + 2.6f, 1.0f});
    // spirogyra.setEnvironment(dish.floorY, dish.ceilY(), dish.radius * 0.6f);
    // spirogyra.addForceGenerator(std::make_unique<GravityForce>(water.gravity));
    // spirogyra.addForceGenerator(std::make_unique<BuoyancyForce>(&water));
    // spirogyra.addForceGenerator(std::make_unique<DragForce>(0.9f));
    const Vector3 groupCenters[NUM_GROUPS] = {
        {3.0f, 1.5f, 0.0f},
        {-1.5f, 3.5f, 2.6f},
        {-1.5f, 2.5f, -2.6f}};

    for (int g = 0; g < NUM_GROUPS; g++)
    {
        BoidGroup &grp = groups[g];
        grp.spawnCenter = groupCenters[g];
        grp.params.separationRadius = 0.18f;
        grp.params.alignmentRadius = 0.8f;
        grp.params.cohesionRadius = 0.8f;
        grp.params.separationWeight = 1.8f;
        grp.params.alignmentWeight = 2.5f;
        grp.params.cohesionWeight = 1.5f;
        grp.params.maxForce = 0.15f;

        grp.flockStates.resize(BOID_MAX);
        grp.hitCooldown.assign(BOID_MAX, 0.0f);

        for (int i = 0; i < BOID_MAX; i++)
        {
            float angle = (float)i / BOID_MAX * 2.0f * PI;
            float r = 0.12f + 0.08f * ((float)(i % 3) / 2.0f);
            float yOff = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * 0.4f;
            Vector3 pos = {
                groupCenters[g].x + r * cosf(angle),
                groupCenters[g].y + yOff,
                groupCenters[g].z + r * sinf(angle)};
            grp.members.push_back(std::make_unique<Bacteria>(pos));
            Bacteria &b = *grp.members.back();
            b.addForceGenerator(std::make_unique<GravityForce>(water.gravity));
            b.addForceGenerator(std::make_unique<BuoyancyForce>(&water));
            b.addForceGenerator(
                std::make_unique<BoidForceGenerator>(
                    &grp.flockStates, i, Bacteria::BODY_NODES, &grp.params));
            if (i >= BOID_INIT)
                b.bsm.state.alive = false;
        }
    }
    // ─────────────────────────────────────────────────────────────────────────

    const int screenWidth = 1080;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Micro-Life 3D");
    Camera3D camera = {{12.0f, 10.0f, 12.0f},
                       {0.0f, dish.ceilY() * 0.5f, 0.0f},
                       {0.0f, 1.0f, 0.0f},
                       45.0f,
                       CAMERA_PERSPECTIVE};

    bool showDebug = false;
    bool fpvMode = false;
    int fpvGroup = 0;
    int fpvIdx = 0;

    auto findNextLive = [&]()
    {
        for (int attempt = 0; attempt < NUM_GROUPS * BOID_MAX; attempt++)
        {
            fpvIdx++;
            if (fpvIdx >= BOID_MAX)
            {
                fpvIdx = 0;
                fpvGroup = (fpvGroup + 1) % NUM_GROUPS;
            }
            if (groups[fpvGroup].members[fpvIdx]->bsm.state.alive)
                return;
        }
    };

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (!fpvMode)
            UpdateCamera(&camera, CAMERA_FREE);

        Vector3 hunter = amoeba.getCenterPosition();
        Vector3 prey = cocci.getCenterPosition();

        Vector3 dxCocci = Vector3Subtract(prey, hunter);
        dxCocci.y = 0.0f;
        float distCocci = Vector3Length(dxCocci);

        // ── boid groups update ────────────────────────────────────────────────
        if (IsKeyDown(KEY_T))
        {
            for (auto &grp : groups)
            {
                grp.params.cohesionWeight = 3.5f;
                grp.params.separationWeight = 0.8f;
                grp.params.alignmentWeight = 2.5f;
            }
        }
        else if (IsKeyDown(KEY_X))
        {
            for (auto &grp : groups)
            {
                grp.params.cohesionWeight = 0.3f;
                grp.params.separationWeight = 4.0f;
                grp.params.alignmentWeight = 0.5f;
            }
        }
        else if (IsKeyDown(KEY_R))
        {
            for (auto &grp : groups)
            {
                grp.params.cohesionWeight = 1.5f;
                grp.params.separationWeight = 1.8f;
                grp.params.alignmentWeight = 2.5f;
            }
        }
        if (IsKeyPressed(KEY_G))
            showDebug = !showDebug;
        if (IsKeyPressed(KEY_F))
        {
            fpvMode = !fpvMode;
            if (fpvMode)
                findNextLive();
        }
        if (fpvMode && IsKeyPressed(KEY_N))
            findNextLive();

        // Combined boid snapshot for amoeba
        std::vector<BoidState> allBoidStates;
        allBoidStates.reserve(NUM_GROUPS * BOID_MAX);

        for (int g = 0; g < NUM_GROUPS; g++)
        {
            BoidGroup &grp = groups[g];

            // Disperse timer — restore normal weights when it expires
            if (grp.disperseTimer > 0.0f)
            {
                grp.disperseTimer -= dt;
                if (grp.disperseTimer <= 0.0f)
                {
                    grp.params.separationWeight = 1.8f;
                    grp.params.cohesionWeight = 1.5f;
                    grp.params.alignmentWeight = 2.5f;
                }
            }

            // Spawn tick: every 1s, 50% chance if ≥2 alive and a slot is free
            grp.spawnTimer -= dt;
            if (grp.spawnTimer <= 0.0f)
            {
                grp.spawnTimer = 3.0f;
                if (grp.liveCount() >= 2 && grp.disperseTimer <= 0.0f)
                {
                    int slot = grp.deadSlot();
                    if (slot >= 0)
                    {
                        Vector3 centroid = Vector3Zero();
                        Vector3 avgVel = Vector3Zero();
                        Vector3 avgHeading = Vector3Zero();
                        int lc = 0;
                        for (auto &b : grp.members)
                        {
                            if (!b->bsm.state.alive)
                                continue;
                            centroid = Vector3Add(centroid, b->getCenterOfMass());
                            avgHeading = Vector3Add(avgHeading, b->getHeading());
                            auto &ns = b->getNodes();
                            for (int n = 0; n < Bacteria::BODY_NODES; n++)
                                avgVel = Vector3Add(avgVel, ns[n].velocity);
                            lc++;
                        }
                        if (lc > 0)
                        {
                            centroid = Vector3Scale(centroid, 1.0f / lc);
                            avgVel = Vector3Scale(avgVel, 1.0f / (lc * Bacteria::BODY_NODES));
                            if (Vector3Length(avgHeading) > 1e-4f)
                                avgHeading = Vector3Normalize(avgHeading);
                            else
                                avgHeading = {0.0f, 0.0f, -1.0f};
                        }
                        else
                        {
                            centroid = grp.spawnCenter;
                            avgHeading = {0.0f, 0.0f, -1.0f};
                        }

                        float ox = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * 0.08f;
                        float oz = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * 0.08f;
                        Vector3 spawnPos = {centroid.x + ox, centroid.y, centroid.z + oz};
                        grp.members[slot]->reset(spawnPos);

                        // orient nodes along group heading so thrust goes the right way
                        {
                            auto &ns = grp.members[slot]->getNodes();
                            Vector3 h = avgHeading;
                            ns[0].position = Vector3Add(spawnPos, Vector3Scale(h, 0.080f));
                            ns[1].position = Vector3Add(spawnPos, Vector3Scale(h, 0.026f));
                            ns[2].position = Vector3Add(spawnPos, Vector3Scale(h, -0.026f));
                            ns[3].position = Vector3Add(spawnPos, Vector3Scale(h, -0.080f));
                            for (int fi = 0; fi < Bacteria::FLAG_NODES; fi++)
                                ns[Bacteria::BODY_NODES + fi].position = Vector3Add(
                                    ns[3].position,
                                    Vector3Scale(h, -(fi + 1) * 0.0045f));
                            grp.members[slot]->bsm.setHeading(h, atan2f(h.x, h.z));
                            for (int n = 0; n < Bacteria::TOTAL_NODES; n++)
                                ns[n].velocity = avgVel;
                        }

                        grp.spawnCount++;
                    }
                }
            }

            nutrientField.update(dt);

            // spirogyra.update(dt);
            // dish.applyBoundary(spirogyra.getNodes(), 0.4f, 0.06f);
            // {
            //     Vector3 sc = spirogyra.getCenterPosition();
            //     float sr = sqrtf(sc.x * sc.x + sc.z * sc.z);
            //     if (sr > dish.radius * 0.9f && sr > 1e-4f)
            //         spirogyra.onWallHit({-sc.x / sr, 0.0f, -sc.z / sr});
            // }

            for (int i = 0; i < BOID_MAX; i++)
            {
                if (grp.members[i]->bsm.state.alive)
                {
                    grp.flockStates[i] = snapshotState(*grp.members[i]);
                    grp.flockStates[i].alive = true;
                    allBoidStates.push_back(grp.flockStates[i]);
                }
                else
                {
                    grp.flockStates[i].alive = false;
                }
            }

            // compute live centroid for disperse leash
            Vector3 grpCentroid = Vector3Zero();
            int liveCnt = 0;
            for (int i = 0; i < BOID_MAX; i++)
            {
                if (grp.members[i]->bsm.state.alive)
                {
                    grpCentroid = Vector3Add(grpCentroid, grp.members[i]->getCenterOfMass());
                    liveCnt++;
                }
            }
            if (liveCnt > 0)
                grpCentroid = Vector3Scale(grpCentroid, 1.0f / liveCnt);

            const float maxDisperse = 1.5f;

            for (int i = 0; i < BOID_MAX; i++)
            {
                if (grp.hitCooldown[i] > 0.0f)
                    grp.hitCooldown[i] -= dt;
                if (!grp.members[i]->bsm.state.alive)
                    continue;

                {
                    Vector3 sensePos = grp.members[i]->getCenterOfMass();
                    float maxConc = nutrientField.maxConcentration();
                    float localConc = nutrientField.concentrationAt(sensePos) / maxConc;
                    Vector3 grad = nutrientField.gradientAt(sensePos);
                    grp.members[i]->bsm.setFoodTarget(grad, localConc);

                    if (localConc > NUTRIENT_FEED_THRESHOLD)
                    {
                        float eaten = nutrientField.feedAt(sensePos, NUTRIENT_BITE * dt);
                        if (eaten > 0.0f)
                            grp.members[i]->bsm.state.feed(localConc * NUTRIENT_FEED_RATE * dt);
                    }
                }

                grp.members[i]->update(dt);
                dish.applyBoundary(grp.members[i]->getNodes());

                Vector3 com = grp.members[i]->getCenterOfMass();

                if (grp.disperseTimer > 0.0f && liveCnt > 0)
                {
                    Vector3 toCentroid = Vector3Subtract(grpCentroid, com);
                    float dist = Vector3Length(toCentroid);
                    if (dist > maxDisperse)
                    {
                        Vector3 leashDir = Vector3Scale(toCentroid, 1.0f / dist);
                        float strength = (dist - maxDisperse) * 2.0f;
                        auto &ns = grp.members[i]->getNodes();
                        for (int n = 0; n < Bacteria::BODY_NODES; n++)
                            ns[n].force = Vector3Add(ns[n].force, Vector3Scale(leashDir, strength));
                    }
                }

                float r = sqrtf(com.x * com.x + com.z * com.z);
                if (r > dish.radius * 0.82f)
                {
                    Vector3 awayDir = {-com.x / r, 0.0f, -com.z / r};
                    grp.members[i]->onWallHit(awayDir);
                }

                Vector3 awayFromPredator = Vector3Subtract(com, hunter);
                float predatorDist = Vector3Length(awayFromPredator);
                if (predatorDist < PREDATOR_AVOID_RADIUS && predatorDist > 1e-4f)
                {
                    float proximity = 1.0f - predatorDist / PREDATOR_AVOID_RADIUS;
                    grp.members[i]->bsm.onPredatorNearby(awayFromPredator, proximity);
                }

                if (predatorDist < BACTERIA_CONSUME_RADIUS)
                {
                    grp.members[i]->bsm.state.alive = false;
                    grp.members[i]->bsm.state.causeOfDeath =
                        InternalState::CauseOfDeath::ATTACK;
                    grp.members[i]->bsm.state.hitCount = 0;
                    grp.members[i]->bsm.state.hitWindowTimer = 0.0f;
                    grp.flockStates[i].alive = false;
                    grp.hitCooldown[i] = 0.0f;
                    amoeba.feed(BACTERIA_NUTRITION);

                    // whole boid disperses when one of its members is eaten
                    grp.disperseTimer = DISPERSE_DUR;
                    grp.params.separationWeight = 4.5f;
                    grp.params.cohesionWeight = 0.1f;
                    grp.params.alignmentWeight = 0.5f;
                }
            }
        }
        // ── inter-flagella collision (no hit, just separation) ────────────────
        {
            std::vector<Bacteria *> live;
            live.reserve(NUM_GROUPS * BOID_MAX);
            for (int g = 0; g < NUM_GROUPS; g++)
                for (auto &b : groups[g].members)
                    if (b->bsm.state.alive)
                        live.push_back(b.get());

            const float cRad = 0.08f;
            for (int a = 0; a < (int)live.size(); a++)
            {
                for (int b = a + 1; b < (int)live.size(); b++)
                {
                    Vector3 ca = live[a]->getCenterOfMass();
                    Vector3 cb = live[b]->getCenterOfMass();
                    Vector3 diff = Vector3Subtract(ca, cb);
                    float dist = Vector3Length(diff);
                    if (dist < cRad && dist > 1e-6f)
                    {
                        Vector3 push = Vector3Scale(diff, 1.0f / dist);
                        float impulse = (cRad - dist) * 3.0f;
                        auto &nsA = live[a]->getNodes();
                        auto &nsB = live[b]->getNodes();
                        for (int n = 0; n < Bacteria::BODY_NODES; n++)
                        {
                            nsA[n].velocity = Vector3Add(nsA[n].velocity, Vector3Scale(push, impulse * 0.5f));
                            nsB[n].velocity = Vector3Add(nsB[n].velocity, Vector3Scale(push, -impulse * 0.5f));
                        }
                    }
                }
            }
        }
        // ─────────────────────────────────────────────────────────────────────

        float amoebaTemp = dish.temperatureAt(hunter);
        Vector3 amoebaTempGradient = dish.temperatureGradientAt(hunter);
        amoeba.actuate(dt, cocci, allBoidStates, amoebaTemp, amoebaTempGradient, AMOEBA_TEMP_TARGET);
        cocci.actuate(dt, dish);

        if (distCocci < 0.667f)
        {
            amoeba.feed(100.0f);
            cocci.respawn(hunter, dish.radius * 0.85f, dish.floorY + 2.0f);
        }

        amoeba.updatePhysicsImplicit(dt);
        cocci.updatePhysicsImplicit(dt);
        dish.applyBoundary(amoeba.getNodes());
        dish.applyBoundary(cocci.getNodes());

        if (fpvMode)
        {
            if (!groups[fpvGroup].members[fpvIdx]->bsm.state.alive)
                findNextLive();
            Bacteria *fpvB = groups[fpvGroup].members[fpvIdx].get();
            Vector3 com = fpvB->getCenterOfMass();
            Vector3 h = fpvB->getHeading();
            Vector3 worldUp = {0.0f, 1.0f, 0.0f};
            camera.position = Vector3Add(com, Vector3Scale(h, 0.04f));
            camera.target = Vector3Add(com, Vector3Scale(h, 0.5f));
            camera.up = worldUp;
            camera.fovy = 70.0f;
        }
        else
        {
            camera.fovy = 45.0f;
        }

        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255});
        BeginMode3D(camera);

        dish.draw(AMOEBA_TEMP_TARGET);
        nutrientField.draw(camera);
        // spirogyra.draw();
        amoeba.draw();
        cocci.draw();
        // for (auto &b : flock)
        //     b.draw(showDebug);
        for (int g = 0; g < NUM_GROUPS; g++)
            for (auto &b : groups[g].members)
                b->draw(showDebug);

        dish.drawShell();

        EndMode3D();

        float camTemp = dish.temperatureAt(camera.position);
        DrawText(TextFormat("Camera: %.1f C", camTemp), 30, 30, 20, RAYWHITE);

        Vector3 amoebaPos = amoeba.getCenterPosition();
        amoebaTemp = dish.temperatureAt(amoebaPos);
        if (isLookingAt(amoebaPos, camera, screenWidth, screenHeight))
            drawAmoebaStatusPanel(amoeba, amoebaTemp, screenWidth);

        if (showDebug)
        {
            for (int g = 0; g < NUM_GROUPS; g++)
            {
                for (int i = 0; i < BOID_MAX; i++)
                {
                    if (!groups[g].members[i]->bsm.state.alive)
                        continue;

                    Vector3 com = groups[g].members[i]->getCenterOfMass();

                    Vector3 camDir = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
                    Vector3 toCom = Vector3Subtract(com, camera.position);
                    if (Vector3DotProduct(camDir, toCom) <= 0.0f)
                        continue;

                    Vector2 sp = GetWorldToScreen(com, camera);
                    if (sp.x < 0 || sp.x > screenWidth || sp.y < 0 || sp.y > screenHeight)
                        continue;

                    int x = (int)sp.x - 30;
                    int y = (int)sp.y - 36;
                    int w = 60, h = 5;

                    DrawRectangle(x, y, w, h, {40, 40, 40, 180});
                    DrawRectangle(x, y, (int)(w * groups[g].members[i]->bsm.state.hunger), h, ORANGE);
                    DrawRectangleLines(x, y, w, h, GRAY);

                    DrawRectangle(x, y + 7, w, h, {40, 40, 40, 180});
                    DrawRectangle(x, y + 7, (int)(w * groups[g].members[i]->bsm.state.fear), h, RED);
                    DrawRectangleLines(x, y + 7, w, h, GRAY);
                }

                char buf[64];
                snprintf(buf, sizeof(buf), "G%d: %d alive  spawned:%d",
                         g, groups[g].liveCount(), groups[g].spawnCount);
                DrawText(buf, 10, screenHeight - 56 + g * 16, 14, RAYWHITE);
            }
        }

        {
            int cx = screenWidth / 2;
            int cy = screenHeight / 2;
            int len = 10;
            int gap = 4;
            Color chColor = {255, 255, 255, 200};
            DrawLine(cx - gap - len, cy, cx - gap, cy, chColor);
            DrawLine(cx + gap, cy, cx + gap + len, cy, chColor);
            DrawLine(cx, cy - gap - len, cx, cy - gap, chColor);
            DrawLine(cx, cy + gap, cx, cy + gap + len, chColor);
            DrawPixel(cx, cy, chColor);
        }

        DrawFPS(10, screenHeight - 24);
        if (fpvMode)
        {
            DrawText(TextFormat("FPV  G%d #%d  |  F=exit  N=next bacterium", fpvGroup, fpvIdx),
                     10, 10, 16, YELLOW);
        }
        else
        {
            DrawText("T=tighten  X=disperse  R=reset  G=debug  F=FPV  |  WASD+mouse=camera",
                     10, 10, 16, RAYWHITE);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
