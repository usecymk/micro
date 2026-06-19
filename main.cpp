// #include <me mory>

#include <vector>
#include <memory>
#include <unordered_map>
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
#include "src/Obstacle.h"
#include "src/ObstaclePerception.h"
#include "src/PopulationManager.h"

static constexpr int   BOID_MAX             = 32;
static constexpr int   BOID_INIT            = 16;
static constexpr int   NUM_GROUPS           = 6;
static constexpr int   TOTAL_BACTERIA       = NUM_GROUPS * BOID_MAX;
static constexpr float DISPERSE_DUR         = 5.0f;
static constexpr float AMOEBA_TEMP_TARGET       = 40.0f;
static constexpr float AMOEBA_TEMP_COMFORT_BAND = 1.25f;
static constexpr float BACTERIA_OPTIMAL_TEMP    = 37.0f;
static constexpr float BACTERIA_HIT_RADIUS  = 0.90f;
static constexpr float PREDATOR_FLEE_RADIUS = 6.5f;
static constexpr float BACTERIA_NUTRITION   = 45.0f;
static constexpr float NUTRIENT_FEED_THRESHOLD = 0.12f;
static constexpr float NUTRIENT_BITE           = 0.018f;
static constexpr float NUTRIENT_FEED_RATE      = 0.315f;
static constexpr float BACTERIA_OBSTACLE_RADIUS = 0.06f;
static constexpr float AMOEBA_OBSTACLE_RADIUS   = 0.12f;
static constexpr float JOIN_RADIUS  = 1.5f;   // detached bacterium joins nearby group
static constexpr float PAIR_RADIUS  = 0.8f;   // two detached bacteria form a new group
static constexpr float MERGE_MULT   = 1.5f;   // mergeRadius = MERGE_MULT * cohesionRadius

// ── helpers ───────────────────────────────────────────────────────────────────

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
    Vector3 camDir  = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    Vector3 toTarget = Vector3Subtract(worldPos, camera.position);
    if (Vector3DotProduct(camDir, toTarget) <= 0.0f) return false;
    Vector2 screenPos = GetWorldToScreen(worldPos, camera);
    if (screenPos.x < 0.0f || screenPos.x > screenWidth ||
        screenPos.y < 0.0f || screenPos.y > screenHeight) return false;
    float dx = screenPos.x - screenWidth  * 0.5f;
    float dy = screenPos.y - screenHeight * 0.5f;
    return std::sqrt(dx * dx + dy * dy) < 95.0f;
}

static void drawStatusBar(int x, int y, int w, int h,
                          float value01, float threshold01,
                          Color fill, const char *label)
{
    value01     = Clamp(value01,     0.0f, 1.0f);
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
    const int x = screenWidth - 285, y = 72, w = 250, barW = 210, barH = 10;
    DrawRectangleRounded({(float)x,(float)y,(float)w,142.0f}, 0.10f, 8, {6,14,24,210});
    DrawRectangleRoundedLines({(float)x,(float)y,(float)w,142.0f}, 0.10f, 8, {95,170,210,180});
    DrawText("Amoeba", x+14, y+10, 18, RAYWHITE);
    DrawText(TextFormat("State: %s", amoeba.getStateName()), x+14, y+32, 13, {215,230,240,255});
    drawStatusBar(x+14, y+62, barW, barH, amoeba.getHunger()/100.0f,
                  amoeba.getHungerThreshold()/100.0f, ORANGE,
                  TextFormat("Hunger %.0f / 100", amoeba.getHunger()));
    float tempMin = 18.0f, tempMax = 50.0f;
    drawStatusBar(x+14, y+91, barW, barH,
                  (currentTemp-tempMin)/(tempMax-tempMin),
                  (AMOEBA_TEMP_TARGET-tempMin)/(tempMax-tempMin),
                  {80,190,255,255}, TextFormat("Temperature %.1f C", currentTemp));
    drawStatusBar(x+14, y+120, barW, barH,
                  amoeba.getTemperatureStress()/100.0f,
                  amoeba.getTemperatureThreshold()/100.0f,
                  {180,120,255,255},
                  TextFormat("Temp stress %.0f / 100", amoeba.getTemperatureStress()));
}

// ── main ──────────────────────────────────────────────────────────────────────

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

    //CocciCluster cocci({-3.0f, dish.floorY + 2.0f, 0.0f});
    std::vector<std::unique_ptr<CocciCluster>> cocciClusters;
    Vector3 cocciSpawnPositions[] = {
        {-3.0f, dish.floorY + 2.0f,  0.0f},
        { 5.0f, dish.floorY + 2.2f,  4.0f},
        {-6.0f, dish.floorY + 1.8f, -5.0f}
    };

    for (const auto& pos : cocciSpawnPositions) {
        auto cluster = std::make_unique<CocciCluster>(pos);
        cluster->addForceGenerator(std::make_unique<DragForce>(0.4f));
        cocciClusters.push_back(std::move(cluster));
    }
    //cocci.addForceGenerator(std::make_unique<DragForce>(0.4f));

    NutrientField nutrientField;
    nutrientField.initAtIsotherm(dish, AMOEBA_TEMP_TARGET, 1100, 2);

    std::vector<Obstacle> obstacles = {
        Obstacle::makeSphere({ 3.5f, dish.floorY + 2.2f,  2.0f}, 0.85f),
        Obstacle::makeSphere({-4.5f, dish.floorY + 3.0f, -3.0f}, 0.55f),
        Obstacle::makeBox({ 0.0f, dish.floorY + 2.0f, -5.5f}, {0.55f, 1.1f, 0.45f}),
        Obstacle::makeBox({-2.5f, dish.floorY + 1.8f,  4.5f}, {0.75f, 0.9f, 0.35f}),
        Obstacle::makeBox({ 5.5f, dish.floorY + 2.8f, -1.0f}, {0.4f,  0.7f, 0.4f})
    };

    // EXPERIMENT: groups packed close together (~1.5-4 units apart).
    // They immediately merge into one swarm, converge on the same food source, and die.
    // const Vector3 groupCenters[NUM_GROUPS] = {
    //     { 3.0f, 1.5f,  0.0f},
    //     {-1.5f, 3.5f,  2.6f},
    //     {-1.5f, 2.5f, -2.6f},
    //     {-3.0f, 1.5f,  0.0f},
    //     { 1.5f, 3.5f, -2.6f},
    //     { 1.5f, 2.5f,  2.6f}
    // };

    // Groups spread at ~radius 9, 60° apart — well beyond the 4.5 merge radius so
    // each colony wanders independently and finds its own food patch.
    const Vector3 groupCenters[NUM_GROUPS] = {
        { 9.0f, 2.0f,  0.0f},   // east
        { 4.5f, 3.2f,  7.8f},   // northeast
        {-4.5f, 1.8f,  7.8f},   // northwest
        {-9.0f, 2.5f,  0.0f},   // west
        {-4.5f, 3.0f, -7.8f},   // southwest
        { 4.5f, 2.2f, -7.8f}    // southeast
    };

    // ── flat global bacteria pool ─────────────────────────────────────────────
    std::vector<std::unique_ptr<Bacteria>> bacteria;
    bacteria.reserve(TOTAL_BACTERIA);
    std::vector<BoidState> flockStates(TOTAL_BACTERIA);
    std::vector<float>     hitCooldown(TOTAL_BACTERIA,    0.0f);
    std::vector<float>     detachCooldown(TOTAL_BACTERIA, 0.0f);
    PopulationManager popManager;

    BoidBehavior boidParams;
    boidParams.separationRadius = 0.18f;
    boidParams.alignmentRadius  = 3.0f;
    boidParams.cohesionRadius   = 3.0f;
    boidParams.separationWeight = 1.8f;
    boidParams.alignmentWeight  = 2.5f;
    boidParams.cohesionWeight   = 1.5f;
    boidParams.maxForce         = 0.15f;

    int nextGroupId = NUM_GROUPS;

    // per-group metadata — keyed by live groupId (dynamic/unbounded)
    std::unordered_map<int, float> spawnTimers;
    std::unordered_map<int, float> disperseTimers;
    std::unordered_map<int, int>   spawnCounts;
    for (int g = 0; g < NUM_GROUPS; g++) {
        spawnTimers[g]    = 1.0f;
        disperseTimers[g] = 0.0f;
        spawnCounts[g]    = 0;
    }

    for (int g = 0; g < NUM_GROUPS; g++)
    {
        for (int i = 0; i < BOID_MAX; i++)
        {
            int     idx   = g * BOID_MAX + i;
            float   angle = (float)i / BOID_INIT * 2.0f * PI;
            float   r     = 0.12f + 0.08f * ((float)(i % 3) / 2.0f);
            float   yOff  = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * 0.4f;
            Vector3 pos   = {
                groupCenters[g].x + r * cosf(angle),
                groupCenters[g].y + yOff,
                groupCenters[g].z + r * sinf(angle)
            };
            bacteria.push_back(std::make_unique<Bacteria>(pos));
            Bacteria &b = *bacteria.back();
            b.addForceGenerator(std::make_unique<GravityForce>(water.gravity));
            b.addForceGenerator(std::make_unique<BuoyancyForce>(&water));
            b.addForceGenerator(std::make_unique<BoidForceGenerator>(
                &flockStates, idx, Bacteria::BODY_NODES, &boidParams));
            flockStates[idx].groupId = g;
            if (i >= BOID_INIT) b.bsm.state.alive = false;
        }
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
    bool fpvMode   = false;
    int  fpvIdx    = 0;

    auto findNextLive = [&]() {
        for (int attempt = 0; attempt < TOTAL_BACTERIA; attempt++) {
            fpvIdx = (fpvIdx + 1) % TOTAL_BACTERIA;
            if (bacteria[fpvIdx]->bsm.state.alive) return;
        }
    };

    Shader amoebaShader = LoadShader("src/amoeba.vs", "src/amoeba.fs");

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (!fpvMode) UpdateCamera(&camera, CAMERA_FREE);

        Vector3 hunter = amoeba.getCenterPosition();
        // Vector3 prey   = cocci.getCenterPosition();
        // Vector3 dxCocci = Vector3Subtract(prey, hunter); dxCocci.y = 0.0f;
        // float distCocci = Vector3Length(dxCocci);

        // ── input ─────────────────────────────────────────────────────────────
        if (IsKeyDown(KEY_T)) {
            boidParams.cohesionWeight = 3.5f; boidParams.separationWeight = 0.8f; boidParams.alignmentWeight = 2.5f;
        } else if (IsKeyDown(KEY_X)) {
            boidParams.cohesionWeight = 0.3f; boidParams.separationWeight = 4.0f; boidParams.alignmentWeight = 0.5f;
        } else if (IsKeyDown(KEY_R)) {
            boidParams.cohesionWeight = 1.5f; boidParams.separationWeight = 1.8f; boidParams.alignmentWeight = 2.5f;
        }
        if (IsKeyPressed(KEY_G)) showDebug = !showDebug;
        if (IsKeyPressed(KEY_F)) { fpvMode = !fpvMode; if (fpvMode) findNextLive(); }
        if (fpvMode && IsKeyPressed(KEY_N)) findNextLive();

        // ── step 1: snapshot all bacteria → flockStates ───────────────────────
        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (!bacteria[idx]->bsm.state.alive) { flockStates[idx].alive = false; continue; }
            int  keepGroup    = flockStates[idx].groupId;
            bool keepDetached = flockStates[idx].detached;
            flockStates[idx]          = snapshotState(*bacteria[idx]);
            flockStates[idx].alive    = true;
            flockStates[idx].groupId  = keepGroup;
            flockStates[idx].detached = keepDetached;
        }

        // ── step 2: per-group centroid / live-count / avg-hunger ─────────────
        std::unordered_map<int, Vector3> grpCentroidSum;
        std::unordered_map<int, int>     grpLiveCount;
        std::unordered_map<int, float>   grpHungerSum;

        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (!flockStates[idx].alive) continue;
            int gid = flockStates[idx].groupId;
            if (gid < 0) continue;
            grpCentroidSum[gid] = Vector3Add(grpCentroidSum[gid], bacteria[idx]->getCenterOfMass());
            grpLiveCount[gid]++;
            grpHungerSum[gid] += bacteria[idx]->bsm.state.hunger;
        }
        std::unordered_map<int, Vector3> grpCentroid;
        std::unordered_map<int, float>   grpAvgHunger;
        for (auto &[gid, cnt] : grpLiveCount) {
            grpCentroid[gid]  = Vector3Scale(grpCentroidSum[gid], 1.0f / cnt);
            grpAvgHunger[gid] = grpHungerSum[gid] / cnt;
        }

        // ── step 3: group merging ─────────────────────────────────────────────
        {
            const float mergeRadius = boidParams.cohesionRadius * MERGE_MULT;
            std::vector<int> gids;
            gids.reserve(grpLiveCount.size());
            for (auto &[gid, _] : grpLiveCount) gids.push_back(gid);

            for (int ai = 0; ai < (int)gids.size(); ai++)
            {
                for (int bi = ai + 1; bi < (int)gids.size(); bi++)
                {
                    int ga = gids[ai], gb = gids[bi];
                    if (!grpCentroid.count(ga) || !grpCentroid.count(gb)) continue;
                    if (Vector3Distance(grpCentroid[ga], grpCentroid[gb]) > mergeRadius) continue;

                    int keep = (grpLiveCount[ga] >= grpLiveCount[gb]) ? ga : gb;
                    int lose = (keep == ga) ? gb : ga;

                    for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
                        if (flockStates[idx].groupId == lose)
                            flockStates[idx].groupId = keep;

                    grpLiveCount[keep]   += grpLiveCount[lose];
                    grpCentroidSum[keep]  = Vector3Add(grpCentroidSum[keep], grpCentroidSum[lose]);
                    grpCentroid[keep]     = Vector3Scale(grpCentroidSum[keep], 1.0f / grpLiveCount[keep]);
                    grpHungerSum[keep]   += grpHungerSum[lose];
                    grpAvgHunger[keep]    = grpHungerSum[keep] / grpLiveCount[keep];

                    grpCentroid.erase(lose);  grpLiveCount.erase(lose);
                    grpHungerSum.erase(lose); grpCentroidSum.erase(lose);
                    disperseTimers.erase(lose);
                    spawnTimers.erase(lose);
                    spawnCounts.erase(lose);

                    gids.erase(gids.begin() + bi);
                    bi--;
                }
            }
        }

        // ── step 4: detached bacteria — rejoin or pair-form ───────────────────
        // phase A: no longer in crisis → re-attach (switch group if closer one nearby)
        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (!flockStates[idx].alive || !flockStates[idx].detached) continue;
            const auto &st = bacteria[idx]->bsm.state;
            if (st.hunger >= 0.60f || st.tempStress >= 0.85f) continue;

            Vector3 myPos   = bacteria[idx]->getCenterOfMass();
            float bestDist  = JOIN_RADIUS;
            int   bestGid   = flockStates[idx].groupId;  // default: re-tether to current group
            for (auto &[gid, centroid] : grpCentroid) {
                float d = Vector3Distance(myPos, centroid);
                if (d < bestDist) { bestDist = d; bestGid = gid; }
            }
            flockStates[idx].groupId  = bestGid;
            flockStates[idx].detached = false;
        }

        // phase B: two detached bacteria within PAIR_RADIUS form a new group
        for (int a = 0; a < TOTAL_BACTERIA; a++)
        {
            if (!flockStates[a].alive || !flockStates[a].detached) continue;
            for (int b = a + 1; b < TOTAL_BACTERIA; b++)
            {
                if (!flockStates[b].alive || !flockStates[b].detached) continue;
                if (Vector3Distance(bacteria[a]->getCenterOfMass(),
                                    bacteria[b]->getCenterOfMass()) > PAIR_RADIUS) continue;
                int newGid = nextGroupId++;
                flockStates[a].groupId  = newGid;  flockStates[a].detached = false;
                flockStates[b].groupId  = newGid;  flockStates[b].detached = false;
                spawnTimers[newGid]    = 3.0f;
                disperseTimers[newGid] = 0.0f;
                spawnCounts[newGid]    = 0;
                break;
            }
        }

        // ── step 5: disperse timers ───────────────────────────────────────────
        for (auto &[gid, timer] : disperseTimers)
            if (timer > 0.0f) timer -= dt;

        // ── step 6: reproduction ──────────────────────────────────────────────
        auto findDeadSlot = [&]() -> int {
            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
                if (!bacteria[idx]->bsm.state.alive) return idx;
            return -1;
        };

        for (auto &[gid, timer] : spawnTimers)
        {
            timer -= dt;
            if (timer > 0.0f) continue;
            timer = 3.0f;

            int lc = grpLiveCount.count(gid) ? grpLiveCount[gid] : 0;
            if (lc < 2 || (disperseTimers.count(gid) && disperseTimers[gid] > 0.0f)) continue;

            int slot = findDeadSlot();
            if (slot < 0) continue;

            Vector3 centroid   = grpCentroid.count(gid) ? grpCentroid[gid] : Vector3Zero();
            Vector3 avgVel     = Vector3Zero();
            Vector3 avgHeading = Vector3Zero();
            int cnt = 0;
            for (int idx = 0; idx < TOTAL_BACTERIA; idx++) {
                if (!bacteria[idx]->bsm.state.alive || flockStates[idx].groupId != gid) continue;
                avgHeading = Vector3Add(avgHeading, bacteria[idx]->getHeading());
                auto &ns = bacteria[idx]->getNodes();
                for (int n = 0; n < Bacteria::BODY_NODES; n++)
                    avgVel = Vector3Add(avgVel, ns[n].velocity);
                cnt++;
            }
            if (cnt > 0) {
                avgVel     = Vector3Scale(avgVel, 1.0f / (cnt * Bacteria::BODY_NODES));
                avgHeading = (Vector3Length(avgHeading) > 1e-4f) ?
                             Vector3Normalize(avgHeading) : Vector3{0.0f, 0.0f, -1.0f};
            } else {
                avgHeading = {0.0f, 0.0f, -1.0f};
            }

            float ox = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * 0.08f;
            float oz = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * 0.08f;
            Vector3 spawnPos = {centroid.x + ox, centroid.y, centroid.z + oz};
            bacteria[slot]->reset(spawnPos);

            {
                auto &ns  = bacteria[slot]->getNodes();
                Vector3 h = avgHeading;
                ns[0].position = Vector3Add(spawnPos, Vector3Scale(h,  0.080f));
                ns[1].position = Vector3Add(spawnPos, Vector3Scale(h,  0.026f));
                ns[2].position = Vector3Add(spawnPos, Vector3Scale(h, -0.026f));
                ns[3].position = Vector3Add(spawnPos, Vector3Scale(h, -0.080f));
                for (int fi = 0; fi < Bacteria::FLAG_NODES; fi++)
                    ns[Bacteria::BODY_NODES + fi].position = Vector3Add(
                        ns[3].position, Vector3Scale(h, -(fi + 1) * 0.0045f));
                bacteria[slot]->bsm.setHeadingAndTarget(h, atan2f(h.x, h.z));
                for (int n = 0; n < Bacteria::TOTAL_NODES; n++)
                    ns[n].velocity = avgVel;
            }

            flockStates[slot].groupId  = gid;
            flockStates[slot].detached = false;
            spawnCounts[gid]++;
        }

        // ── step 7: nutrient field (once per frame) ───────────────────────────
        nutrientField.update(dt);

        // ── step 8: per-bacterium update ──────────────────────────────────────
        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (hitCooldown[idx]    > 0.0f) hitCooldown[idx]    -= dt;
            if (detachCooldown[idx] > 0.0f) detachCooldown[idx] -= dt;
            if (!bacteria[idx]->bsm.state.alive) continue;

            int     gid      = flockStates[idx].groupId;
            int     liveCnt  = grpLiveCount.count(gid)  ? grpLiveCount[gid]  : 0;
            Vector3 centroid = grpCentroid.count(gid)   ? grpCentroid[gid]   : Vector3Zero();
            float   avgHung  = grpAvgHunger.count(gid)  ? grpAvgHunger[gid]  : 1.0f;
            float   dispT    = disperseTimers.count(gid) ? disperseTimers[gid] : 0.0f;

            // sense food & obstacles
            {
                Vector3 sensePos  = bacteria[idx]->getCenterOfMass();
                Vector3 bHeading  = bacteria[idx]->getHeading();
                float   maxConc   = nutrientField.maxConcentration();
                float   localConc = nutrientField.concentrationAt(sensePos) / maxConc;
                Vector3 grad      = nutrientField.bestFoodDirection(sensePos);
                bacteria[idx]->bsm.setFoodTarget(grad, localConc);

                AttentionMode attention = AttentionMode::WANDER;
                if (bacteria[idx]->bsm.state.fear > 0.35f)       attention = AttentionMode::FEARFUL;
                else if (bacteria[idx]->bsm.state.hunger > 0.7f)  attention = AttentionMode::HUNGRY;

                ObstacleSenseParams obsParams;
                obsParams.senseRadius     = 2.8f;
                obsParams.attentionRadius = 1.4f;
                obsParams.criticalRadius  = 0.35f;
                applySelectiveAttention(obsParams, attention);

                ObstacleSenseResult obsSense = senseObstacles(
                    sensePos, bHeading, obstacles, obsParams, BACTERIA_OBSTACLE_RADIUS, attention);
                if (obsSense.detected)
                    bacteria[idx]->bsm.setObstacleSense(obsSense.avoidDirection, obsSense.urgency);
                else
                    bacteria[idx]->bsm.setObstacleSense({0.0f, 0.0f, 0.0f}, 0.0f);

                if (localConc > NUTRIENT_FEED_THRESHOLD) {
                    float eaten = nutrientField.feedAt(sensePos, NUTRIENT_BITE * dt);
                    if (eaten > 0.0f) {
                        float satiation = (eaten / maxConc) * NUTRIENT_FEED_RATE * dt;
                        bacteria[idx]->bsm.state.feed(satiation);
                    }
                }
            }

            Vector3 com         = bacteria[idx]->getCenterOfMass();
            Vector3 awayFromHunter = Vector3Subtract(com, hunter);
            float   predatorDist   = Vector3Length(awayFromHunter);
            if (predatorDist < PREDATOR_FLEE_RADIUS && predatorDist > 1e-4f)
            {
                float proximity = 1.0f - Clamp(predatorDist / PREDATOR_FLEE_RADIUS, 0.0f, 1.0f);
                bacteria[idx]->bsm.onPredatorNearby(
                    Vector3Scale(awayFromHunter, 1.0f / predatorDist), proximity);
            }

            float   ambientTemp = dish.temperatureAt(com);
            Vector3 tempGrad    = dish.temperatureGradientAt(com);
            float   rDist       = sqrtf(com.x * com.x + com.z * com.z);
            bool    nearWall    = rDist > dish.radius * 0.82f;
            bacteria[idx]->update(dt, ambientTemp, tempGrad, nearWall);
            dish.applyBoundary(bacteria[idx]->getNodes());
            resolveObstacleCollisions(obstacles, bacteria[idx]->getNodes(), BACTERIA_OBSTACLE_RADIUS);

            com = bacteria[idx]->getCenterOfMass();

            // critical detachment
            {
                const InternalState &st = bacteria[idx]->bsm.state;
                bool criticalHunger = st.hunger    >= 0.60f;
                bool criticalTemp   = st.tempStress >= 0.85f;
                if (!flockStates[idx].detached) {
                    if (criticalHunger) {
                        flockStates[idx].detached = true;
                    } else if (criticalTemp && detachCooldown[idx] <= 0.0f) {
                        flockStates[idx].detached = true;
                        detachCooldown[idx] = 8.0f;
                    }
                }
            }

            // tether: velocity impulse toward group centroid for attached bacteria
            if (!flockStates[idx].detached && liveCnt > 0)
            {
                Vector3 toCentroid = Vector3Subtract(centroid, com);
                float   dist       = Vector3Length(toCentroid);
                float   t          = Clamp((avgHung - 0.25f) / 0.45f, 0.0f, 1.0f);
                float   deadZone   = dispT > 0.0f ? 1.5f  : (0.4f + (1.0f - t) * 1.6f);
                float   tetherK    = dispT > 0.0f ? 4.0f  : (4.0f + t * 11.0f);
                if (dist > deadZone)
                {
                    float   strength = std::min((dist - deadZone) * tetherK * dt, 0.04f);
                    Vector3 leashDir = Vector3Normalize(toCentroid);
                    auto   &ns       = bacteria[idx]->getNodes();
                    for (int n = 0; n < Bacteria::BODY_NODES; n++)
                        ns[n].velocity = Vector3Add(ns[n].velocity, Vector3Scale(leashDir, strength));
                }
            }
        }

        // ── step 9: inter-bacterium body separation ───────────────────────────
        {
            std::vector<Bacteria*> live;
            live.reserve(TOTAL_BACTERIA);
            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
                if (bacteria[idx]->bsm.state.alive) live.push_back(bacteria[idx].get());

            const float cRad = 0.08f;
            for (int a = 0; a < (int)live.size(); a++)
            {
                for (int b = a + 1; b < (int)live.size(); b++)
                {
                    Vector3 ca   = live[a]->getCenterOfMass();
                    Vector3 cb   = live[b]->getCenterOfMass();
                    Vector3 diff = Vector3Subtract(ca, cb);
                    float   dist = Vector3Length(diff);
                    if (dist < cRad && dist > 1e-6f)
                    {
                        Vector3 push    = Vector3Scale(diff, 1.0f / dist);
                        float   impulse = (cRad - dist) * 3.0f;
                        auto   &nsA     = live[a]->getNodes();
                        auto   &nsB     = live[b]->getNodes();
                        for (int n = 0; n < Bacteria::BODY_NODES; n++)
                        {
                            nsA[n].velocity = Vector3Add(nsA[n].velocity, Vector3Scale(push,  impulse * 0.5f));
                            nsB[n].velocity = Vector3Add(nsB[n].velocity, Vector3Scale(push, -impulse * 0.5f));
                        }
                    }
                }
            }
        }

        // ── step 10: second snapshot (post-physics, for amoeba) ───────────────
        std::vector<BoidState> allBoidStates;
        allBoidStates.reserve(TOTAL_BACTERIA);
        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (!bacteria[idx]->bsm.state.alive) continue;
            int  keepGroup    = flockStates[idx].groupId;
            bool keepDetached = flockStates[idx].detached;
            flockStates[idx]          = snapshotState(*bacteria[idx]);
            flockStates[idx].alive    = true;
            flockStates[idx].groupId  = keepGroup;
            flockStates[idx].detached = keepDetached;
            allBoidStates.push_back(flockStates[idx]);
        }

        float amoebaTemp = dish.temperatureAt(hunter);
        Vector3 amoebaTempGradient = dish.temperatureGradientAt(hunter);
        
        amoeba.actuate(dt, cocciClusters, allBoidStates, amoebaTemp, amoebaTempGradient,
                       AMOEBA_TEMP_TARGET, AMOEBA_TEMP_COMFORT_BAND, obstacles);
        amoeba.updatePhysicsImplicit(dt);
        dish.applyBoundary(amoeba.getNodes());
        resolveObstacleCollisions(obstacles, amoeba.getNodes(), AMOEBA_OBSTACLE_RADIUS);

        {
            Vector3 hunterAfter = amoeba.getCenterPosition();
            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
            {
                if (hitCooldown[idx] > 0.0f) continue;
                if (!bacteria[idx]->bsm.state.alive) continue;

                Vector3 com = bacteria[idx]->getCenterOfMass();
                Vector3 away = Vector3Subtract(com, hunterAfter);
                float   dist = Vector3Length(away);
                if (dist >= BACTERIA_HIT_RADIUS) continue;

                if (dist > 1e-4f)
                    bacteria[idx]->bsm.onPredatorNearby(Vector3Scale(away, 1.0f / dist), 1.0f);
                bacteria[idx]->bsm.state.onAttackHit();
                flockStates[idx].alive = bacteria[idx]->bsm.state.alive;
                if (!bacteria[idx]->bsm.state.alive) amoeba.feed(BACTERIA_NUTRITION);
                hitCooldown[idx] = 1.5f;

                int gid = flockStates[idx].groupId;
                flockStates[idx].detached = true;
                detachCooldown[idx] = std::max(detachCooldown[idx], DISPERSE_DUR);
                if (gid >= 0) disperseTimers[gid] = DISPERSE_DUR;
            }
        }

        // Process loop iterations cleanly for each tracked Cocci cluster instance
        for (auto& cocci : cocciClusters)
        {
            cocci->actuate(dt, dish);
            
            // Check specific proximity thresholds to trigger amoeba feeding / cluster relocation response
            float currentDist = Vector3Distance(cocci->getCenterPosition(), hunter);
            if (currentDist < 0.667f) 
            { 
                amoeba.feed(100.0f); 
                cocci->respawn(hunter, dish.radius * 0.85f, dish.floorY + 2.0f); 
            }
            
            cocci->updatePhysicsImplicit(dt);
            dish.applyBoundary(cocci->getNodes());
            resolveObstacleCollisions(obstacles, cocci->getNodes(), 0.08f);
        }

        // DEBUG: amoeba disabled
        // float amoebaTemp = dish.temperatureAt(hunter);
        // Vector3 amoebaTempGradient = dish.temperatureGradientAt(hunter);
        // amoeba.actuate(dt, cocci, allBoidStates, amoebaTemp, amoebaTempGradient, AMOEBA_TEMP_TARGET, obstacles);
        // cocci.actuate(dt, dish);
        // if (distCocci < 0.667f) { amoeba.feed(100.0f); cocci.respawn(hunter, dish.radius*0.85f, dish.floorY+2.0f); }
        // amoeba.updatePhysicsImplicit(dt);
        // cocci.updatePhysicsImplicit(dt);
        // dish.applyBoundary(amoeba.getNodes());
        // dish.applyBoundary(cocci.getNodes());
        // resolveObstacleCollisions(obstacles, amoeba.getNodes(), AMOEBA_OBSTACLE_RADIUS);
        // resolveObstacleCollisions(obstacles, cocci.getNodes(), 0.08f);

        // ── FPV camera ────────────────────────────────────────────────────────
        if (fpvMode)
        {
            if (!bacteria[fpvIdx]->bsm.state.alive) findNextLive();
            Bacteria *fpvB = bacteria[fpvIdx].get();
            Vector3   com  = fpvB->getCenterOfMass();
            Vector3   h    = fpvB->getHeading();
            camera.position = Vector3Add(com, Vector3Scale(h,  0.04f));
            camera.target   = Vector3Add(com, Vector3Scale(h,  0.5f));
            camera.up       = {0.0f, 1.0f, 0.0f};
            camera.fovy     = 70.0f;
        }
        else
        {
            camera.fovy = 45.0f;
        }

        popManager.update(bacteria, GetFrameTime());

        // ── draw ──────────────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255}); // Keeps the micro-world aesthetic
        
        BeginMode3D(camera);

            // Draw environmental assets
            dish.draw();
            for (const auto &obs : obstacles) obs.draw();
            nutrientField.draw(camera);

            if (showDebug)
            {
                dish.drawFloorIsotherm(AMOEBA_TEMP_TARGET, {255, 150, 55, 190});
                dish.drawFloorIsotherm(BACTERIA_OPTIMAL_TEMP, {80, 200, 255, 190});
            }
            
            // Cleanly render the shader-managed Amoeba within the open 3D context
            float time = (float)GetTime();
            amoeba.draw(amoebaShader, camera.position, time, false);
            
            
            // Draw remaining biological entities inside 3D mode
            for (const auto& cocci : cocciClusters) 
            {
                cocci->draw();
            }

            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
                bacteria[idx]->draw(showDebug);

            // Debug arrows for bacteria[0]
            if (showDebug && bacteria[0]->bsm.state.alive)
            {
                Vector3 dbPos = bacteria[0]->getCenterOfMass();
                Vector3 grad  = nutrientField.bestFoodDirection(dbPos);
                float   gLen  = Vector3Length(grad);
                if (gLen > 1e-5f) {
                    Vector3 gDir = Vector3Scale(grad, 1.0f / gLen);
                    DrawLine3D(dbPos, Vector3Add(dbPos, Vector3Scale(gDir, 0.6f)), GREEN);
                    DrawSphere(Vector3Add(dbPos, Vector3Scale(gDir, 0.6f)), 0.03f, GREEN);
                }
                DrawLine3D(dbPos, Vector3Add(dbPos, Vector3Scale(bacteria[0]->getHeading(), 0.4f)), WHITE);
            }

            dish.drawShell();
            
        EndMode3D(); // Cleanly close 3D mode once

        // ── HUD (2D overlays drawn on top) ───────────────────────────────────
        float camTemp = dish.temperatureAt(camera.position);
        DrawText(TextFormat("Camera: %.1f C", camTemp), 30, 30, 20, RAYWHITE);

        // Restored Amoeba Status Panel overlay
        drawAmoebaStatusPanel(amoeba, amoebaTemp, screenWidth);

        // bacteria[0] state panel — debug only
        if (showDebug)
        {
            DrawText("Temp zones (mid-liquid):", 10, screenHeight - 72, 14, RAYWHITE);
            DrawText(TextFormat("  orange = %.0f C (amoeba)", AMOEBA_TEMP_TARGET),
                     10, screenHeight - 56, 14, {255, 150, 55, 255});
            DrawText(TextFormat("  blue   = %.0f C (bacteria)", BACTERIA_OPTIMAL_TEMP),
                     10, screenHeight - 40, 14, {80, 200, 255, 255});

            if (bacteria[0]->bsm.state.alive)
            {
                Bacteria   &db = *bacteria[0];
                const auto &st = db.bsm.state;

                const char *behaviorName = "?";
                switch (db.bsm.behavior) {
                    case Behavior::WANDER:         behaviorName = "WANDER";         break;
                    case Behavior::SEEK_FOOD:      behaviorName = "SEEK_FOOD";      break;
                    case Behavior::ESCAPE:         behaviorName = "ESCAPE";         break;
                    case Behavior::SEEK_TEMP:      behaviorName = "SEEK_TEMP";      break;
                    case Behavior::AVOID_OBSTACLE: behaviorName = "AVOID_OBSTACLE"; break;
                }

                int px = 10, py = 55;
                DrawRectangle(px - 4, py - 4, 300, 145, {0, 0, 0, 160});
                DrawText(TextFormat("Behavior: %s",  behaviorName),     px, py,       14, YELLOW);
                DrawText(TextFormat("Hunger:   %.2f", st.hunger),       px, py + 18,  14, ORANGE);
                DrawText(TextFormat("Fear:     %.2f", st.fear),         px, py + 34,  14, RED);
                DrawText(TextFormat("TmpStress:%.2f", st.tempStress),   px, py + 50,  14, {80,180,255,255});

                float bTemp = dish.temperatureAt(db.getCenterOfMass());
                DrawText(TextFormat("Amb temp: %.1f C", bTemp),         px, py + 66,  14, {200,200,200,255});
                DrawText(TextFormat("nearWall: %s",
                    [&]{ float r2 = db.getCenterOfMass().x * db.getCenterOfMass().x +
                                    db.getCenterOfMass().z * db.getCenterOfMass().z;
                         return sqrtf(r2) > dish.radius * 0.82f ? "YES" : "no"; }()),
                    px, py + 82, 14, {200,200,200,255});

                Vector3 dbPos     = db.getCenterOfMass();
                float   maxConc   = nutrientField.maxConcentration();
                float   localConc = nutrientField.concentrationAt(dbPos) / maxConc;
                Vector3 foodDir   = nutrientField.bestFoodDirection(dbPos);
                float   foodLen   = Vector3Length(foodDir);
                DrawText(TextFormat("localConc:%.3f", localConc), px, py + 98, 14, {180,255,180,255});
                if (foodLen > 1e-5f)
                    DrawText(TextFormat("foodDir: (%.2f, %.2f, %.2f)", foodDir.x, foodDir.y, foodDir.z),
                             px, py + 114, 14, {180,255,180,255});
                else
                    DrawText("foodDir: zero (will wander)", px, py + 114, 14, RED);
            }
            else
            {
                DrawText("Bacterium[0] DEAD", 10, 55, 14, RED);
            }
        }

        if (showDebug)
        {
            // per-bacterium status bars
            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
            {
                if (!bacteria[idx]->bsm.state.alive) continue;
                Vector3 com = bacteria[idx]->getCenterOfMass();
                Vector3 camDir = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
                if (Vector3DotProduct(camDir, Vector3Subtract(com, camera.position)) <= 0.0f) continue;
                Vector2 sp = GetWorldToScreen(com, camera);
                if (sp.x < 0 || sp.x > screenWidth || sp.y < 0 || sp.y > screenHeight) continue;

                int x = (int)sp.x - 30, y = (int)sp.y - 43, w = 60, h = 5;
                DrawRectangle(x, y,      w, h, {40,40,40,180});
                DrawRectangle(x, y,      (int)(w * bacteria[idx]->bsm.state.hunger),    h, ORANGE);
                DrawRectangleLines(x, y, w, h, GRAY);
                DrawRectangle(x, y+7,      w, h, {40,40,40,180});
                DrawRectangle(x, y+7,      (int)(w * bacteria[idx]->bsm.state.fear),    h, RED);
                DrawRectangleLines(x, y+7, w, h, GRAY);
                DrawRectangle(x, y+14,      w, h, {40,40,40,180});
                DrawRectangle(x, y+14,      (int)(w * bacteria[idx]->bsm.state.tempStress), h, {80,180,255,255});
                DrawRectangleLines(x, y+14, w, h, GRAY);
            }

            // group stats (up to 8 groups)
            int shown = 0;
            for (auto &[gid, cnt] : grpLiveCount) {
                if (shown >= 8) break;
                DrawText(TextFormat("G%d: %d alive  spawned:%d", gid, cnt,
                                    spawnCounts.count(gid) ? spawnCounts[gid] : 0),
                         10, screenHeight - 40 - shown * 16, 14, RAYWHITE);
                shown++;
            }
        }

        // crosshair
        {
            int cx = screenWidth/2, cy = screenHeight/2, len = 10, gap = 4;
            Color cc = {255,255,255,200};
            DrawLine(cx-gap-len, cy, cx-gap,     cy, cc);
            DrawLine(cx+gap,     cy, cx+gap+len, cy, cc);
            DrawLine(cx, cy-gap-len, cx, cy-gap,     cc);
            DrawLine(cx, cy+gap,     cx, cy+gap+len, cc);
            DrawPixel(cx, cy, cc);
        }

        DrawFPS(10, screenHeight - 24);
        if (fpvMode)
            DrawText(TextFormat("FPV #%d  |  F=exit  N=next", fpvIdx), 10, 10, 16, YELLOW);
        else
            DrawText("T=tighten  X=disperse  R=reset  G=debug  F=FPV  |  WASD+mouse=camera",
                     10, 10, 16, RAYWHITE);

        EndDrawing();
    }

    UnloadShader(amoebaShader);
    CloseWindow();
    return 0;
}