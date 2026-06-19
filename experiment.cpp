
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

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
#include "src/PetriDish.h"
#include "src/BoidBehavior.h"
#include "src/Obstacle.h"
#include "src/ObstaclePerception.h"
#include "src/PopulationManager.h"

static constexpr int   BOID_MAX              = 32;
static constexpr int   BOID_INIT             = 16;
static constexpr int   NUM_GROUPS            = 6;
static constexpr int   TOTAL_BACTERIA        = NUM_GROUPS * BOID_MAX;
static constexpr float DISPERSE_DUR          = 5.0f;
static constexpr float AMOEBA_TEMP_TARGET    = 40.0f;
static constexpr float BACTERIA_HIT_RADIUS   = 0.75f;
static constexpr float BACTERIA_NUTRITION    = 45.0f;
static constexpr float NUTRIENT_FEED_THRESHOLD = 0.18f;
static constexpr float NUTRIENT_BITE           = 0.9f;
static constexpr float NUTRIENT_FEED_RATE      = 0.8f;
static constexpr float BACTERIA_OBSTACLE_RADIUS  = 0.06f;
static constexpr float AMOEBA_OBSTACLE_RADIUS    = 0.12f;
static constexpr float JOIN_RADIUS             = 1.5f;
static constexpr float PAIR_RADIUS             = 0.8f;
static constexpr float MERGE_MULT              = 1.5f;
static constexpr float PREDATOR_SENSE_RADIUS   = 3.5f;
static constexpr float PAIR_COOLDOWN           = 8.0f;
static constexpr float LOG_INTERVAL            = 0.5f;

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

static Color groupColor(int gid)
{
    static const Color palette[] = {
        {80,  200, 255, 220},
        {255, 120,  80, 220},
        {120, 255, 100, 220},
        {255, 220,  60, 220},
        {200, 100, 255, 220},
        {255,  80, 160, 220},
        {100, 255, 220, 220},
        {255, 180, 100, 220},
    };
    return palette[((gid % 8) + 8) % 8];
}

static const char *behaviorLabel(Behavior b)
{
    switch (b)
    {
        case Behavior::WANDER:         return "WANDER";
        case Behavior::SEEK_FOOD:      return "SEEK_FOOD";
        case Behavior::ESCAPE:         return "ESCAPE";
        case Behavior::SEEK_TEMP:      return "SEEK_TEMP";
        case Behavior::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        default:                       return "?";
    }
}

// ── experiment metrics

struct ExperimentMetrics
{
    bool  recording       = true;
    bool  paused          = false;
    float simTime         = 0.0f;
    float logTimer        = 0.0f;
    int   screenshotCount = 0;

    int predatorHits      = 0;
    int pairFormations    = 0;
    int disperseEvents    = 0;
    int peakGroupCount    = NUM_GROUPS;

    std::ofstream timeseries;
    std::ofstream events;
    std::unordered_set<uint64_t> loggedMerges;

    static uint64_t mergeKey(int keep, int lose)
    {
        return ((uint64_t)(uint32_t)keep << 32) | (uint32_t)lose;
    }

    void openLogs()
    {
        timeseries.open("experiment_scatter_log.csv");
        events.open("experiment_events.csv");
        if (timeseries.is_open())
        {
            timeseries << "time_s,active_groups,satellite_groups,detached_count,"
                          "dispersing_groups,live_bacteria,predator_hits,pair_formations\n";
        }
        if (events.is_open())
            events << "time_s,event,group_id,detail\n";
    }

    void closeLogs()
    {
        if (timeseries.is_open()) timeseries.close();
        if (events.is_open()) events.close();
    }

    void logEvent(float t, const char *name, int gid = -1, const char *detail = "")
    {
        if (!recording || !events.is_open()) return;
        events << t << "," << name << "," << gid << "," << detail << "\n";
    }

    void logSample(float t,
                   int activeGroups,
                   int satelliteGroups,
                   int detachedCount,
                   int dispersingGroups,
                   int liveBacteria)
    {
        if (!recording || !timeseries.is_open()) return;
        timeseries << t << ","
                   << activeGroups << ","
                   << satelliteGroups << ","
                   << detachedCount << ","
                   << dispersingGroups << ","
                   << liveBacteria << ","
                   << predatorHits << ","
                   << pairFormations << "\n";
    }
};

static int countActiveGroups(const std::unordered_map<int, int> &grpLiveCount)
{
    return (int)grpLiveCount.size();
}

static int countSatelliteGroups(const std::unordered_map<int, int> &grpLiveCount)
{
    int n = 0;
    for (const auto &[gid, cnt] : grpLiveCount)
        if (gid >= NUM_GROUPS && cnt > 0) n++;
    return n;
}

static int countDetached(const std::vector<BoidState> &flockStates)
{
    int n = 0;
    for (const auto &s : flockStates)
        if (s.alive && s.detached) n++;
    return n;
}

static int countDispersing(const std::unordered_map<int, float> &disperseTimers)
{
    int n = 0;
    for (const auto &[gid, timer] : disperseTimers)
        if (timer > 0.0f) n++;
    return n;
}

static int countLiveBacteria(const std::vector<std::unique_ptr<Bacteria>> &bacteria)
{
    int n = 0;
    for (const auto &b : bacteria)
        if (b->bsm.state.alive) n++;
    return n;
}

static const char *experimentPhase(int predatorHits,
                                   int dispersingGroups,
                                   int satelliteGroups)
{
    if (predatorHits == 0) return "BASELINE — six independent colonies";
    if (dispersingGroups > 0) return "SCATTER — predator hit, colonies dispersing";
    if (satelliteGroups > 0) return "SATELLITE — new groups formed from pairs";
    return "REORGANIZATION — colonies recovering";
}

static void drawExperimentPanel(const ExperimentMetrics &m,
                                int activeGroups,
                                int satelliteGroups,
                                int detachedCount,
                                int dispersingGroups,
                                int liveBacteria,
                                const Amoeba &amoeba,
                                float amoebaTemp,
                                int screenWidth)
{
    const int x = screenWidth - 320;
    const int y = 10;
    const int w = 305;
    const int h = 210;

    DrawRectangleRounded({(float)x, (float)y, (float)w, (float)h}, 0.08f, 8, {6, 14, 24, 220});
    DrawRectangleRoundedLines({(float)x, (float)y, (float)w, (float)h}, 0.08f, 8, {95, 170, 210, 200});

    int ty = y + 10;
    auto line = [&](const char *text, int size, Color color) {
        DrawText(text, x + 12, ty, size, color);
        ty += size + 5;
    };

    line("Experiment: Predator Scatter", 15, RAYWHITE);
    line(experimentPhase(m.predatorHits, dispersingGroups, satelliteGroups), 12, {180, 220, 255, 255});
    line(TextFormat("Sim time: %.1f s%s", m.simTime, m.paused ? "  [PAUSED]" : ""), 12, GRAY);
    line(TextFormat("Active groups: %d  (peak %d)", activeGroups, m.peakGroupCount), 13, RAYWHITE);
    line(TextFormat("Satellite groups: %d  |  Detached: %d", satelliteGroups, detachedCount), 13, {120, 255, 180, 255});
    line(TextFormat("Dispersing: %d  |  Live bacteria: %d", dispersingGroups, liveBacteria), 13, RAYWHITE);
    line(TextFormat("Predator hits: %d  |  Pair events: %d", m.predatorHits, m.pairFormations), 13, ORANGE);
    line(TextFormat("Amoeba: %s  hunger %.0f", amoeba.getStateName(), amoeba.getHunger()), 12, {215, 230, 240, 255});
    line(TextFormat("Recording: %s", m.recording ? "ON (CSV)" : "OFF"), 12, m.recording ? GREEN : RED);
}

// ── main

int main()
{
    PetriDish dish;

    FluidEnvironment water;
    water.density  = 1000.0f;
    water.surfaceY = dish.ceilY();
    water.gravity  = {0.0f, -9.81f, 0.0f};

    Amoeba amoeba({7.0f, dish.floorY + 2.5f, 0.5f});
    amoeba.setFloorY(dish.floorY);
    amoeba.addForceGenerator(std::make_unique<DragForce>(0.6f));
    amoeba.feed(-70.0f);

    std::vector<std::unique_ptr<CocciCluster>> cocciClusters;

    NutrientField nutrientField;
    nutrientField.init(dish.radius, dish.floorY, dish.ceilY(), 1400, 7);

    std::vector<Obstacle> obstacles = {
        Obstacle::makeSphere({ 3.5f, dish.floorY + 2.2f,  2.0f}, 0.85f),
        Obstacle::makeSphere({-4.5f, dish.floorY + 3.0f, -3.0f}, 0.55f),
        Obstacle::makeBox({ 0.0f, dish.floorY + 2.0f, -5.5f}, {0.55f, 1.1f, 0.45f}),
        Obstacle::makeBox({-2.5f, dish.floorY + 1.8f,  4.5f}, {0.75f, 0.9f, 0.35f}),
        Obstacle::makeBox({ 5.5f, dish.floorY + 2.8f, -1.0f}, {0.4f,  0.7f, 0.4f})
    };

    const Vector3 groupCenters[NUM_GROUPS] = {
        { 9.0f, 2.0f,  0.0f},
        { 4.5f, 3.2f,  7.8f},
        {-4.5f, 1.8f,  7.8f},
        {-9.0f, 2.5f,  0.0f},
        {-4.5f, 3.0f, -7.8f},
        { 4.5f, 2.2f, -7.8f}
    };

    std::vector<std::unique_ptr<Bacteria>> bacteria;
    bacteria.reserve(TOTAL_BACTERIA);
    std::vector<BoidState> flockStates(TOTAL_BACTERIA);
    std::vector<float>     hitCooldown(TOTAL_BACTERIA,    0.0f);
    std::vector<float>     detachCooldown(TOTAL_BACTERIA, 0.0f);
    std::vector<float>     pairCooldown(TOTAL_BACTERIA,    0.0f);
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

    std::unordered_map<int, float> spawnTimers;
    std::unordered_map<int, float> disperseTimers;
    std::unordered_map<int, int>   spawnCounts;
    for (int g = 0; g < NUM_GROUPS; g++)
    {
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

    const int screenWidth  = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight,
               "Micro-Life 3D — Experiment: Predator Scatter");
    Camera3D camera = {{14.0f, 12.0f, 14.0f},
                       {0.0f, dish.ceilY() * 0.5f, 0.0f},
                       {0.0f, 1.0f, 0.0f},
                       45.0f,
                       CAMERA_PERSPECTIVE};

    bool showDebug = true;
    bool fpvMode   = false;
    int  fpvIdx    = 0;

    auto findNextLive = [&]() {
        for (int attempt = 0; attempt < TOTAL_BACTERIA; attempt++)
        {
            fpvIdx = (fpvIdx + 1) % TOTAL_BACTERIA;
            if (bacteria[fpvIdx]->bsm.state.alive) return;
        }
    };

    Shader amoebaShader = LoadShader("src/amoeba.vs", "src/amoeba.fs");

    ExperimentMetrics metrics;
    metrics.openLogs();
    metrics.logEvent(0.0f, "EXPERIMENT_START", -1, "perimeter_colonies_hungry_amoeba");

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        float rawDt = GetFrameTime();
        if (IsKeyPressed(KEY_SPACE)) metrics.paused = !metrics.paused;
        if (IsKeyPressed(KEY_G)) showDebug = !showDebug;
        if (IsKeyPressed(KEY_F)) { fpvMode = !fpvMode; if (fpvMode) findNextLive(); }
        if (fpvMode && IsKeyPressed(KEY_N)) findNextLive();
        if (IsKeyPressed(KEY_P)) metrics.recording = !metrics.recording;
        if (IsKeyPressed(KEY_R))
        {
            boidParams.cohesionWeight   = 1.5f;
            boidParams.separationWeight = 1.8f;
            boidParams.alignmentWeight  = 2.5f;
        }
        if (IsKeyPressed(KEY_S))
        {
            std::string path = "experiment_screenshot_" + std::to_string(metrics.screenshotCount++) + ".png";
            TakeScreenshot(path.c_str());
        }

        float dt = metrics.paused ? 0.0f : rawDt;
        if (!fpvMode && !metrics.paused) UpdateCamera(&camera, CAMERA_FREE);

        if (!metrics.paused)
            metrics.simTime += rawDt;

        Vector3 hunter = amoeba.getCenterPosition();

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
        for (auto &[gid, cnt] : grpLiveCount)
        {
            grpCentroid[gid]  = Vector3Scale(grpCentroidSum[gid], 1.0f / cnt);
            grpAvgHunger[gid] = grpHungerSum[gid] / cnt;
        }

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

                    {
                        uint64_t key = ExperimentMetrics::mergeKey(keep, lose);
                        if (metrics.loggedMerges.insert(key).second)
                        {
                            metrics.logEvent(metrics.simTime, "MERGE", keep,
                                             TextFormat("absorbed_group_%d", lose));
                        }
                    }

                    gids.erase(gids.begin() + bi);
                    bi--;
                }
            }
        }

        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (!flockStates[idx].alive || !flockStates[idx].detached) continue;
            const auto &st = bacteria[idx]->bsm.state;
            if (st.hunger >= 0.60f || st.tempStress >= 0.85f) continue;

            Vector3 myPos   = bacteria[idx]->getCenterOfMass();
            float bestDist  = JOIN_RADIUS;
            int   bestGid   = -1;
            for (auto &[gid, centroid] : grpCentroid)
            {
                float d = Vector3Distance(myPos, centroid);
                if (d < bestDist) { bestDist = d; bestGid = gid; }
            }
            if (bestGid < 0) continue;

            flockStates[idx].groupId  = bestGid;
            flockStates[idx].detached = false;
            metrics.logEvent(metrics.simTime, "REJOIN", bestGid, TextFormat("bacterium_%d", idx));
        }

        for (int a = 0; a < TOTAL_BACTERIA; a++)
        {
            if (!flockStates[a].alive || !flockStates[a].detached) continue;
            if (pairCooldown[a] > 0.0f) continue;
            for (int b = a + 1; b < TOTAL_BACTERIA; b++)
            {
                if (!flockStates[b].alive || !flockStates[b].detached) continue;
                if (pairCooldown[b] > 0.0f) continue;
                if (Vector3Distance(bacteria[a]->getCenterOfMass(),
                                    bacteria[b]->getCenterOfMass()) > PAIR_RADIUS) continue;

                int newGid = nextGroupId++;
                flockStates[a].groupId  = newGid;  flockStates[a].detached = false;
                flockStates[b].groupId  = newGid;  flockStates[b].detached = false;
                pairCooldown[a] = PAIR_COOLDOWN;
                pairCooldown[b] = PAIR_COOLDOWN;
                spawnTimers[newGid]    = 3.0f;
                disperseTimers[newGid] = 0.0f;
                spawnCounts[newGid]    = 0;

                metrics.pairFormations++;
                metrics.logEvent(metrics.simTime, "SATELLITE_PAIR", newGid,
                                 TextFormat("bacteria_%d_%d", a, b));
                break;
            }
        }

        for (auto &[gid, timer] : disperseTimers)
            if (timer > 0.0f) timer -= dt;

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
            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
            {
                if (!bacteria[idx]->bsm.state.alive || flockStates[idx].groupId != gid) continue;
                avgHeading = Vector3Add(avgHeading, bacteria[idx]->getHeading());
                auto &ns = bacteria[idx]->getNodes();
                for (int n = 0; n < Bacteria::BODY_NODES; n++)
                    avgVel = Vector3Add(avgVel, ns[n].velocity);
                cnt++;
            }
            if (cnt > 0)
            {
                avgVel     = Vector3Scale(avgVel, 1.0f / (cnt * Bacteria::BODY_NODES));
                avgHeading = (Vector3Length(avgHeading) > 1e-4f)
                    ? Vector3Normalize(avgHeading) : Vector3{0.0f, 0.0f, -1.0f};
            }
            else
            {
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

        nutrientField.update(dt);

        // ── step 8: per-bacterium update ──────────────────────────────────────
        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (hitCooldown[idx]    > 0.0f) hitCooldown[idx]    -= dt;
            if (detachCooldown[idx] > 0.0f) detachCooldown[idx] -= dt;
            if (pairCooldown[idx]   > 0.0f) pairCooldown[idx]   -= dt;
            if (!bacteria[idx]->bsm.state.alive) continue;

            int     gid      = flockStates[idx].groupId;
            int     liveCnt  = grpLiveCount.count(gid)  ? grpLiveCount[gid]  : 0;
            Vector3 centroid = grpCentroid.count(gid)   ? grpCentroid[gid]   : Vector3Zero();
            float   avgHung  = grpAvgHunger.count(gid)  ? grpAvgHunger[gid]  : 1.0f;
            float   dispT    = disperseTimers.count(gid) ? disperseTimers[gid] : 0.0f;

            {
                Vector3 sensePos  = bacteria[idx]->getCenterOfMass();
                Vector3 bHeading  = bacteria[idx]->getHeading();
                float   maxConc   = nutrientField.maxConcentration();
                float   localConc = nutrientField.concentrationAt(sensePos) / maxConc;
                Vector3 grad      = nutrientField.bestFoodDirection(sensePos);
                bacteria[idx]->bsm.setFoodTarget(grad, localConc);

                AttentionMode attention = AttentionMode::WANDER;
                if (bacteria[idx]->bsm.state.fear > 0.35f)      attention = AttentionMode::FEARFUL;
                else if (bacteria[idx]->bsm.state.hunger > 0.7f) attention = AttentionMode::HUNGRY;

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

                if (localConc > NUTRIENT_FEED_THRESHOLD)
                {
                    float eaten = nutrientField.feedAt(sensePos, NUTRIENT_BITE * dt);
                    if (eaten > 0.0f)
                        bacteria[idx]->bsm.state.feed(localConc * NUTRIENT_FEED_RATE * dt);
                }
            }

            Vector3 com         = bacteria[idx]->getCenterOfMass();
            float   ambientTemp = dish.temperatureAt(com);
            Vector3 tempGrad    = dish.temperatureGradientAt(com);
            float   rDist       = sqrtf(com.x * com.x + com.z * com.z);
            bool    nearWall    = rDist > dish.radius * 0.82f;

            // Proximity fear — flee before contact (not wired in main.cpp).
            float predatorDist = Vector3Distance(com, hunter);
            if (predatorDist < PREDATOR_SENSE_RADIUS)
            {
                Vector3 away = Vector3Subtract(com, hunter);
                float prox = Clamp(1.0f - predatorDist / PREDATOR_SENSE_RADIUS, 0.0f, 1.0f);
                bacteria[idx]->bsm.onPredatorNearby(away, prox);
            }

            bacteria[idx]->update(dt, ambientTemp, tempGrad, nearWall);
            dish.applyBoundary(bacteria[idx]->getNodes());
            resolveObstacleCollisions(obstacles, bacteria[idx]->getNodes(), BACTERIA_OBSTACLE_RADIUS);

            com = bacteria[idx]->getCenterOfMass();

            {
                const InternalState &st = bacteria[idx]->bsm.state;
                bool criticalHunger = st.hunger    >= 0.60f;
                bool criticalTemp   = st.tempStress >= 0.85f;
                if (!flockStates[idx].detached)
                {
                    if (criticalHunger)
                        flockStates[idx].detached = true;
                    else if (criticalTemp && detachCooldown[idx] <= 0.0f)
                    {
                        flockStates[idx].detached = true;
                        detachCooldown[idx] = 8.0f;
                    }
                }
            }

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

            if (hitCooldown[idx] <= 0.0f && predatorDist < BACTERIA_HIT_RADIUS)
            {
                bacteria[idx]->bsm.state.onAttackHit();
                flockStates[idx].alive = bacteria[idx]->bsm.state.alive;
                if (!bacteria[idx]->bsm.state.alive) amoeba.feed(BACTERIA_NUTRITION);
                hitCooldown[idx] = 1.5f;

                flockStates[idx].detached = true;
                detachCooldown[idx] = std::max(detachCooldown[idx], DISPERSE_DUR);
                if (gid >= 0)
                {
                    bool wasDispersing = disperseTimers.count(gid) && disperseTimers[gid] > 0.0f;
                    disperseTimers[gid] = DISPERSE_DUR;
                    if (!wasDispersing)
                    {
                        metrics.disperseEvents++;
                        metrics.logEvent(metrics.simTime, "DISPERSE", gid,
                                         TextFormat("hit_bacterium_%d", idx));
                    }
                }

                metrics.predatorHits++;
                metrics.logEvent(metrics.simTime, "PREDATOR_HIT", gid,
                                 bacteria[idx]->bsm.state.alive ? "survived" : "killed");
            }
        }

        // ── step 9: inter-bacterium separation ────────────────────────────────
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

        // ── step 10: amoeba ───────────────────────────────────────────────────
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
                       AMOEBA_TEMP_TARGET, obstacles);
        amoeba.updatePhysicsImplicit(dt);
        dish.applyBoundary(amoeba.getNodes());
        resolveObstacleCollisions(obstacles, amoeba.getNodes(), AMOEBA_OBSTACLE_RADIUS);


        grpCentroidSum.clear();
        grpLiveCount.clear();
        for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
        {
            if (!bacteria[idx]->bsm.state.alive) continue;
            int gid = flockStates[idx].groupId;
            if (gid < 0) continue;
            grpCentroidSum[gid] = Vector3Add(grpCentroidSum[gid], bacteria[idx]->getCenterOfMass());
            grpLiveCount[gid]++;
        }
        grpCentroid.clear();
        for (auto &[gid, cnt] : grpLiveCount)
            grpCentroid[gid] = Vector3Scale(grpCentroidSum[gid], 1.0f / cnt);

        int activeGroups     = countActiveGroups(grpLiveCount);
        int satelliteGroups  = countSatelliteGroups(grpLiveCount);
        int detachedCount    = countDetached(flockStates);
        int dispersingGroups = countDispersing(disperseTimers);
        int liveBacteria     = countLiveBacteria(bacteria);
        metrics.peakGroupCount = std::max(metrics.peakGroupCount, activeGroups);

        metrics.logTimer += rawDt;
        if (metrics.logTimer >= LOG_INTERVAL)
        {
            metrics.logTimer = 0.0f;
            metrics.logSample(metrics.simTime, activeGroups, satelliteGroups,
                              detachedCount, dispersingGroups, liveBacteria);
        }

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

        BeginDrawing();
        ClearBackground((Color){5, 10, 20, 255});

        BeginMode3D(camera);
            dish.draw(AMOEBA_TEMP_TARGET);
            for (const auto &obs : obstacles) obs.draw();
            nutrientField.draw(camera);

            for (const auto &[gid, centroid] : grpCentroid)
            {
                Color c = groupColor(gid);
                float pulse = disperseTimers.count(gid) && disperseTimers.at(gid) > 0.0f ? 0.35f : 0.18f;
                DrawSphere(centroid, pulse, Fade(c, 0.35f));
                DrawSphere(centroid, 0.10f, c);
                if (gid >= NUM_GROUPS)
                    DrawCubeWires(centroid, 0.35f, 0.35f, 0.35f, c);
            }

            float time = (float)GetTime();
            amoeba.draw(amoebaShader, camera.position, time, false);

            for (int idx = 0; idx < TOTAL_BACTERIA; idx++)
                bacteria[idx]->draw(showDebug);

            if (showDebug && bacteria[fpvIdx]->bsm.state.alive)
            {
                Bacteria &db = *bacteria[fpvIdx];
                Vector3 dbPos = db.getCenterOfMass();
                Vector3 grad  = nutrientField.bestFoodDirection(dbPos);
                float   gLen  = Vector3Length(grad);
                if (gLen > 1e-5f)
                {
                    Vector3 gDir = Vector3Scale(grad, 1.0f / gLen);
                    DrawLine3D(dbPos, Vector3Add(dbPos, Vector3Scale(gDir, 0.6f)), GREEN);
                }
                if (Vector3Length(db.bsm.getFleeDirection()) > 0.1f)
                {
                    Vector3 flee = db.bsm.getFleeDirection();
                    DrawLine3D(dbPos, Vector3Add(dbPos, Vector3Scale(flee, 0.7f)), RED);
                }
                DrawLine3D(dbPos, Vector3Add(dbPos, Vector3Scale(db.getHeading(), 0.4f)), WHITE);
            }

            dish.drawShell();
        EndMode3D();

        drawExperimentPanel(metrics, activeGroups, satelliteGroups, detachedCount,
                            dispersingGroups, liveBacteria, amoeba, amoebaTemp, screenWidth);

        if (showDebug && bacteria[fpvIdx]->bsm.state.alive)
        {
            const auto &st = bacteria[fpvIdx]->bsm.state;
            int px = 10, py = 55;
            DrawRectangle(px - 4, py - 4, 310, 130, {0, 0, 0, 160});
            DrawText(TextFormat("FPV bacterium #%d  group %d%s", fpvIdx,
                                flockStates[fpvIdx].groupId,
                                flockStates[fpvIdx].detached ? "  [DETACHED]" : ""),
                     px, py, 14, YELLOW);
            DrawText(TextFormat("Behavior: %s", behaviorLabel(bacteria[fpvIdx]->bsm.behavior)),
                     px, py + 18, 14, RAYWHITE);
            DrawText(TextFormat("Hunger %.2f  Fear %.2f  TmpStress %.2f",
                                st.hunger, st.fear, st.tempStress),
                     px, py + 36, 14, RAYWHITE);
        }

        DrawFPS(10, screenHeight - 24);
        DrawText("G=debug  F/N=FPV  P=record  S=screenshot  SPACE=pause  |  WASD+mouse=camera",
                 10, screenHeight - 44, 14, RAYWHITE);

        EndDrawing();
    }

    metrics.logEvent(metrics.simTime, "EXPERIMENT_END", -1, "");
    metrics.closeLogs();

    UnloadShader(amoebaShader);
    CloseWindow();
    return 0;
}
