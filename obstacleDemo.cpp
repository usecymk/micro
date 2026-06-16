

#include <vector>
#include <memory>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <raylib.h>
#include <raymath.h>

#include "src/PetriDish.h"
#include "src/FluidEnvironment.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/DragForce.h"
#include "src/Bacteria.h"
#include "src/Amoeba.h"
#include "src/Cocci.h"
#include "src/BoidBehavior.h"
#include "src/Obstacle.h"
#include "src/ObstaclePerception.h"

static constexpr float BACTERIA_BODY_PAD = 0.06f;
static constexpr float AMOEBA_BODY_PAD   = 0.12f;
static constexpr int   SCREEN_W          = 1280;
static constexpr int   SCREEN_H          = 720;
static constexpr int   PANEL_W           = 300;

enum class DemoScenario
{
    BACTERIA_HEADON,
    BACTERIA_SWARM,
    AMOEBA_NAV,
    COMBINED
};

enum class TuneTarget
{
    BACTERIA,
    AMOEBA
};

struct DemoConfig
{
    ObstacleSenseParams bacteriaParams = {2.8f, 1.4f, 0.35f, 1.0f};
    ObstacleSenseParams amoebaParams    = {6.5f, 3.0f, 0.55f, 1.0f};
    AttentionMode bacteriaAttention     = AttentionMode::WANDER;
    AttentionMode amoebaAttention       = AttentionMode::WANDER;
    bool autoBacteriaAttention          = true;
    bool autoAmoebaAttention            = true;
};

struct ExperimentStats
{
    bool  recording   = false;
    float elapsed     = 0.0f;
    int   frames      = 0;

    int bacteriaAvoidFrames     = 0;
    int amoebaAvoidFrames       = 0;
    int bacteriaCollisionFrames = 0;
    int amoebaCollisionFrames   = 0;

    float minBacteriaClearance = 1e9f;
    float minAmoebaClearance   = 1e9f;
    float sumBacteriaUrgency   = 0.0f;
    float sumAmoebaUrgency     = 0.0f;
    int   bacteriaSamples      = 0;
    int   amoebaSamples        = 0;

    void reset()
    {
        elapsed = 0.0f;
        frames = 0;
        bacteriaAvoidFrames = 0;
        amoebaAvoidFrames = 0;
        bacteriaCollisionFrames = 0;
        amoebaCollisionFrames = 0;
        minBacteriaClearance = 1e9f;
        minAmoebaClearance = 1e9f;
        sumBacteriaUrgency = 0.0f;
        sumAmoebaUrgency = 0.0f;
        bacteriaSamples = 0;
        amoebaSamples = 0;
    }

    void tick(float dt)
    {
        if (!recording) return;
        elapsed += dt;
        frames++;
    }

    void noteBacteria(bool avoiding, float urgency, float clearance)
    {
        if (!recording) return;
        bacteriaSamples++;
        sumBacteriaUrgency += urgency;
        if (avoiding) bacteriaAvoidFrames++;
        if (clearance < 0.0f) bacteriaCollisionFrames++;
        minBacteriaClearance = std::min(minBacteriaClearance, clearance);
    }

    void noteAmoeba(bool avoiding, float urgency, float clearance)
    {
        if (!recording) return;
        amoebaSamples++;
        sumAmoebaUrgency += urgency;
        if (avoiding) amoebaAvoidFrames++;
        if (clearance < 0.0f) amoebaCollisionFrames++;
        minAmoebaClearance = std::min(minAmoebaClearance, clearance);
    }

    float bacteriaAvoidPct() const
    {
        return frames > 0 ? 100.0f * bacteriaAvoidFrames / frames : 0.0f;
    }

    float amoebaAvoidPct() const
    {
        return frames > 0 ? 100.0f * amoebaAvoidFrames / frames : 0.0f;
    }

    float avgBacteriaUrgency() const
    {
        return bacteriaSamples > 0 ? sumBacteriaUrgency / bacteriaSamples : 0.0f;
    }

    float avgAmoebaUrgency() const
    {
        return amoebaSamples > 0 ? sumAmoebaUrgency / amoebaSamples : 0.0f;
    }
};

struct AgentSenseDebug
{
    ObstacleSenseResult sense;
    AttentionMode       mode = AttentionMode::WANDER;
};

static const char *scenarioName(DemoScenario s)
{
    switch (s)
    {
        case DemoScenario::BACTERIA_HEADON: return "Bacteria head-on";
        case DemoScenario::BACTERIA_SWARM: return "Bacteria swarm corridor";
        case DemoScenario::AMOEBA_NAV:     return "Amoeba navigation";
        case DemoScenario::COMBINED:        return "Combined";
    }
    return "?";
}

static const char *attentionName(AttentionMode m)
{
    switch (m)
    {
        case AttentionMode::WANDER:  return "Wander";
        case AttentionMode::HUNGRY:  return "Hungry";
        case AttentionMode::FEARFUL: return "Fearful";
    }
    return "?";
}

static void orientBacteria(Bacteria &b, Vector3 spawnPos, Vector3 heading)
{
    b.reset(spawnPos);
    Vector3 h = Vector3Length(heading) > 1e-4f ? Vector3Normalize(heading) : Vector3{0.0f, 0.0f, 1.0f};

    auto &ns = b.getNodes();
    ns[0].position = Vector3Add(spawnPos, Vector3Scale(h,  0.080f));
    ns[1].position = Vector3Add(spawnPos, Vector3Scale(h,  0.026f));
    ns[2].position = Vector3Add(spawnPos, Vector3Scale(h, -0.026f));
    ns[3].position = Vector3Add(spawnPos, Vector3Scale(h, -0.080f));
    for (int fi = 0; fi < Bacteria::FLAG_NODES; fi++)
        ns[Bacteria::BODY_NODES + fi].position = Vector3Add(
            ns[3].position, Vector3Scale(h, -(fi + 1) * 0.0045f));

    b.bsm.setHeading(h, atan2f(h.x, h.z));
    for (int n = 0; n < Bacteria::TOTAL_NODES; n++)
        ns[n].velocity = Vector3Zero();
}

static std::vector<Obstacle> makeObstacles(const PetriDish &dish, DemoScenario scenario)
{
    float fy = dish.floorY;
    switch (scenario)
    {
        case DemoScenario::BACTERIA_HEADON:
            return {
                Obstacle::makeSphere({0.0f, fy + 2.5f, 0.0f}, 1.0f),
            };

        case DemoScenario::BACTERIA_SWARM:
            return {
                Obstacle::makeBox({-0.5f, fy + 2.2f,  2.2f}, {0.5f, 1.4f, 0.45f}),
                Obstacle::makeBox({-0.5f, fy + 2.2f, -2.2f}, {0.5f, 1.4f, 0.45f}),
                Obstacle::makeSphere({3.5f, fy + 2.0f, 0.0f}, 0.7f),
            };

        case DemoScenario::AMOEBA_NAV:
            return {
                Obstacle::makeSphere({0.5f, fy + 2.4f,  1.5f}, 0.9f),
                Obstacle::makeBox({2.0f, fy + 2.0f, -1.0f}, {0.55f, 1.2f, 0.5f}),
                Obstacle::makeBox({-1.5f, fy + 2.8f, 0.0f}, {0.4f, 0.8f, 0.4f}),
            };

        case DemoScenario::COMBINED:
        default:
            return {
                Obstacle::makeSphere({1.0f, fy + 2.3f, 0.0f}, 0.85f),
                Obstacle::makeBox({-2.0f, fy + 2.0f,  2.5f}, {0.6f, 1.0f, 0.35f}),
                Obstacle::makeBox({-2.0f, fy + 2.0f, -2.5f}, {0.6f, 1.0f, 0.35f}),
            };
    }
}

static void attachAgentForces(std::vector<std::unique_ptr<Bacteria>> &bacteria,
                              std::unique_ptr<Amoeba> &amoeba,
                              std::vector<BoidState> &flockStates,
                              BoidBehavior &boidParams,
                              FluidEnvironment &water)
{
    for (size_t i = 0; i < bacteria.size(); i++)
    {
        bacteria[i]->addForceGenerator(std::make_unique<GravityForce>(water.gravity));
        bacteria[i]->addForceGenerator(std::make_unique<BuoyancyForce>(&water));
        if (flockStates.size() > 1)
        {
            bacteria[i]->addForceGenerator(std::make_unique<BoidForceGenerator>(
                &flockStates, (int)i, Bacteria::BODY_NODES, &boidParams));
        }
    }
    if (amoeba)
        amoeba->addForceGenerator(std::make_unique<DragForce>(0.6f));
}

static void setupScenario(DemoScenario scenario,
                          const PetriDish &dish,
                          std::vector<std::unique_ptr<Bacteria>> &bacteria,
                          std::unique_ptr<Amoeba> &amoeba,
                          std::vector<Obstacle> &obstacles,
                          std::vector<BoidState> &flockStates,
                          BoidBehavior &boidParams,
                          FluidEnvironment &water)
{
    bacteria.clear();
    amoeba.reset();
    flockStates.clear();
    obstacles = makeObstacles(dish, scenario);

    float fy = dish.floorY;
    float midY = fy + dish.height * 0.5f;

    switch (scenario)
    {
        case DemoScenario::BACTERIA_HEADON:
        {
            bacteria.push_back(std::make_unique<Bacteria>(Vector3{-6.0f, midY, 0.0f}));
            orientBacteria(*bacteria[0], {-6.0f, midY, 0.0f}, {1.0f, 0.0f, 0.0f});
            break;
        }

        case DemoScenario::BACTERIA_SWARM:
        {
            const int n = 8;
            flockStates.resize(n);
            for (int i = 0; i < n; i++)
            {
                float z = -2.5f + (float)i / (n - 1) * 5.0f;
                Vector3 pos = {-7.0f, midY + ((float)(i % 3) - 1.0f) * 0.15f, z};
                bacteria.push_back(std::make_unique<Bacteria>(pos));
                orientBacteria(*bacteria.back(), pos, {1.0f, 0.0f, 0.0f});
            }
            break;
        }

        case DemoScenario::AMOEBA_NAV:
        {
            amoeba = std::make_unique<Amoeba>(Vector3{-6.0f, midY, 0.0f});
            amoeba->setFloorY(dish.floorY);
            break;
        }

        case DemoScenario::COMBINED:
        {
            flockStates.resize(3);
            for (int i = 0; i < 3; i++)
            {
                Vector3 pos = {-6.0f, midY, -1.5f + i * 1.5f};
                bacteria.push_back(std::make_unique<Bacteria>(pos));
                orientBacteria(*bacteria.back(), pos, {1.0f, 0.0f, 0.0f});
            }
            amoeba = std::make_unique<Amoeba>(Vector3{-6.0f, midY, 0.0f});
            amoeba->setFloorY(dish.floorY);
            break;
        }
    }

    attachAgentForces(bacteria, amoeba, flockStates, boidParams, water);
}

static AttentionMode bacteriaAttentionFor(const Bacteria &b, const DemoConfig &cfg)
{
    if (!cfg.autoBacteriaAttention)
        return cfg.bacteriaAttention;
    if (b.bsm.state.fear > 0.35f)
        return AttentionMode::FEARFUL;
    if (b.bsm.state.hunger > 0.7f)
        return AttentionMode::HUNGRY;
    return AttentionMode::WANDER;
}

static AttentionMode amoebaAttentionFor(const Amoeba &a, const DemoConfig &cfg)
{
    if (!cfg.autoAmoebaAttention)
        return cfg.amoebaAttention;
    if (a.getHunger() > a.getHungerThreshold())
        return AttentionMode::HUNGRY;
    return AttentionMode::WANDER;
}

static ObstacleSenseResult senseForBacteria(Bacteria &b,
                                            const std::vector<Obstacle> &obstacles,
                                            const DemoConfig &cfg,
                                            AttentionMode mode)
{
    ObstacleSenseParams p = cfg.bacteriaParams;
    applySelectiveAttention(p, mode);
    return senseObstacles(
        b.getCenterOfMass(), b.getHeading(), obstacles, p, BACTERIA_BODY_PAD, mode);
}

static float nearestClearance(Vector3 pos,
                              const std::vector<Obstacle> &obstacles,
                              float bodyPad)
{
    float best = 1e9f;
    for (const auto &obs : obstacles)
        best = std::min(best, obs.signedDistanceToSurface(pos) - bodyPad);
    return best;
}

static void drawSenseDebug(Vector3 pos,
                           Vector3 heading,
                           const ObstacleSenseResult &sense,
                           const ObstacleSenseParams &baseParams,
                           AttentionMode mode,
                           Color senseCol,
                           Color attnCol)
{
    ObstacleSenseParams p = baseParams;
    applySelectiveAttention(p, mode);

    DrawSphereWires(pos, p.senseRadius, 12, 8, senseCol);
    DrawSphereWires(pos, p.attentionRadius, 10, 6, attnCol);
    DrawSphereWires(pos, p.criticalRadius + BACTERIA_BODY_PAD, 8, 4, RED);

    if (sense.detected)
    {
        Vector3 tip = Vector3Add(pos, Vector3Scale(sense.avoidDirection, 1.2f + sense.urgency));
        DrawLine3D(pos, tip, ORANGE);
        DrawSphere(tip, 0.06f, ORANGE);
    }

    if (Vector3Length(heading) > 1e-4f)
    {
        DrawLine3D(pos, Vector3Add(pos, Vector3Scale(Vector3Normalize(heading), 0.5f)), SKYBLUE);
    }
}

static void adjustParam(float &v, float delta, float lo, float hi)
{
    v = Clamp(v + delta, lo, hi);
}

static void drawPanel(int x, int y,
                      DemoScenario scenario,
                      TuneTarget tuneTarget,
                      const DemoConfig &cfg,
                      const ExperimentStats &stats,
                      bool showDebug,
                      bool showHelp)
{
    DrawRectangle(x, 0, PANEL_W, SCREEN_H, {12, 18, 28, 235});
    DrawLine(x, 0, x, SCREEN_H, {60, 90, 120, 255});

    int ty = 16;
    auto line = [&](const char *txt, int size = 14, Color col = RAYWHITE) {
        DrawText(txt, x + 12, ty, size, col);
        ty += size + 6;
    };

    line("OBSTACLE AVOIDANCE DEMO", 16, {120, 210, 255, 255});
    ty += 4;
    line(TextFormat("Scenario: %s", scenarioName(scenario)), 13, LIGHTGRAY);
    line(TextFormat("Tuning: %s", tuneTarget == TuneTarget::BACTERIA ? "Bacteria" : "Amoeba"), 13, LIGHTGRAY);
    line(showDebug ? "Debug overlay: ON" : "Debug overlay: OFF", 13, GRAY);

    ty += 8;
    line("--- Perception params ---", 13, {100, 180, 220, 255});

    const ObstacleSenseParams &tp = tuneTarget == TuneTarget::BACTERIA
        ? cfg.bacteriaParams : cfg.amoebaParams;
    line(TextFormat("Sense radius:     %.2f  [Q/A]", tp.senseRadius));
    line(TextFormat("Attention radius: %.2f  [W/S]", tp.attentionRadius));
    line(TextFormat("Critical radius:  %.2f  [E/D]", tp.criticalRadius));
    line(TextFormat("Attention weight: %.2f  [R/F]", tp.attentionWeight));

    ty += 4;
    line("--- Attention mode ---", 13, {100, 180, 220, 255});
    if (tuneTarget == TuneTarget::BACTERIA)
    {
        line(cfg.autoBacteriaAttention ? "Auto (from hunger/fear) [V]" : "Manual [V]", 13, YELLOW);
        if (!cfg.autoBacteriaAttention)
            line(TextFormat("Mode: %s [Z/X/C]", attentionName(cfg.bacteriaAttention)));
    }
    else
    {
        line(cfg.autoAmoebaAttention ? "Auto (from hunger) [V]" : "Manual [V]", 13, YELLOW);
        if (!cfg.autoAmoebaAttention)
            line(TextFormat("Mode: %s [Z/X/C]", attentionName(cfg.amoebaAttention)));
    }

    ty += 8;
    line("--- Experiment ---", 13, {100, 180, 220, 255});
    line(stats.recording ? "RECORDING [P pause]" : "Paused [P record]", 13,
         stats.recording ? RED : GREEN);
    line(TextFormat("Time: %.1fs  Frames: %d", stats.elapsed, stats.frames), 13, LIGHTGRAY);

    if (stats.frames > 0)
    {
        line(TextFormat("Bact avoid: %.1f%%  coll: %d",
                        stats.bacteriaAvoidPct(), stats.bacteriaCollisionFrames), 12, ORANGE);
        line(TextFormat("Bact min clearance: %.3f", stats.minBacteriaClearance), 12, ORANGE);
        line(TextFormat("Bact avg urgency: %.3f", stats.avgBacteriaUrgency()), 12, ORANGE);

        if (scenario == DemoScenario::AMOEBA_NAV || scenario == DemoScenario::COMBINED)
        {
            line(TextFormat("Amoeba avoid: %.1f%%  coll: %d",
                            stats.amoebaAvoidPct(), stats.amoebaCollisionFrames), 12, {35, 215, 170, 255});
            line(TextFormat("Amoeba min clearance: %.3f", stats.minAmoebaClearance), 12, {35, 215, 170, 255});
            line(TextFormat("Amoeba avg urgency: %.3f", stats.avgAmoebaUrgency()), 12, {35, 215, 170, 255});
        }
    }

    line("[O] reset stats  [L] export CSV", 12, GRAY);

    if (showHelp)
    {
        ty += 8;
        line("--- Controls ---", 13, {100, 180, 220, 255});
        line("[1-4] Scenarios", 12, GRAY);
        line("[Tab] Bacteria/Amoeba tune", 12, GRAY);
        line("[Space] Reset scenario", 12, GRAY);
        line("[G] Debug  [H] Help", 12, GRAY);
        line("[5] Force hungry  [6] Force fearful", 12, GRAY);
        line("WASD+mouse = camera", 12, GRAY);
    }
}

static void exportCsv(const char *path,
                      DemoScenario scenario,
                      const DemoConfig &cfg,
                      const ExperimentStats &stats)
{
    std::ofstream out(path, std::ios::app);
    if (!out) return;

    static bool header = false;
    if (!header)
    {
        out << "scenario,elapsed,frames,bact_avoid_pct,bact_collisions,bact_min_clearance,bact_avg_urgency,"
            << "amoeba_avoid_pct,amoeba_collisions,amoeba_min_clearance,amoeba_avg_urgency,"
            << "bact_sense,bact_attention,bact_critical,bact_weight,"
            << "amoeba_sense,amoeba_attention,amoeba_critical,amoeba_weight\n";
        header = true;
    }

    out << scenarioName(scenario) << ","
        << stats.elapsed << ","
        << stats.frames << ","
        << stats.bacteriaAvoidPct() << ","
        << stats.bacteriaCollisionFrames << ","
        << stats.minBacteriaClearance << ","
        << stats.avgBacteriaUrgency() << ","
        << stats.amoebaAvoidPct() << ","
        << stats.amoebaCollisionFrames << ","
        << stats.minAmoebaClearance << ","
        << stats.avgAmoebaUrgency() << ","
        << cfg.bacteriaParams.senseRadius << ","
        << cfg.bacteriaParams.attentionRadius << ","
        << cfg.bacteriaParams.criticalRadius << ","
        << cfg.bacteriaParams.attentionWeight << ","
        << cfg.amoebaParams.senseRadius << ","
        << cfg.amoebaParams.attentionRadius << ","
        << cfg.amoebaParams.criticalRadius << ","
        << cfg.amoebaParams.attentionWeight << "\n";
}

int main()
{
    PetriDish dish(12.0f, 5.0f, 0.0f);

    FluidEnvironment water;
    water.density  = 1000.0f;
    water.surfaceY = dish.ceilY();
    water.gravity  = {0.0f, -9.81f, 0.0f};

    DemoScenario scenario = DemoScenario::BACTERIA_HEADON;
    DemoConfig   config;
    TuneTarget   tuneTarget = TuneTarget::BACTERIA;
    ExperimentStats stats;

    std::vector<std::unique_ptr<Bacteria>> bacteria;
    std::unique_ptr<Amoeba> amoeba;
    std::vector<Obstacle> obstacles;
    std::vector<BoidState> flockStates;
    BoidBehavior boidParams;
    boidParams.separationRadius = 0.18f;
    boidParams.alignmentRadius  = 0.8f;
    boidParams.cohesionRadius   = 0.8f;
    boidParams.separationWeight = 1.8f;
    boidParams.alignmentWeight  = 2.5f;
    boidParams.cohesionWeight   = 1.5f;
    boidParams.maxForce         = 0.15f;

    CocciCluster cocci({99.0f, dish.floorY + 2.0f, 99.0f});
    cocci.addForceGenerator(std::make_unique<DragForce>(0.4f));

    setupScenario(scenario, dish, bacteria, amoeba, obstacles, flockStates, boidParams, water);

    InitWindow(SCREEN_W, SCREEN_H, "Obstacle Avoidance Demo");
    Camera3D camera = {{8.0f, 6.0f, 10.0f},
                       {0.0f, dish.ceilY() * 0.5f, 0.0f},
                       {0.0f, 1.0f, 0.0f},
                       45.0f,
                       CAMERA_PERSPECTIVE};

    bool showDebug = true;
    bool showHelp  = true;

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        UpdateCamera(&camera, CAMERA_FREE);

        ObstacleSenseParams &activeParams = tuneTarget == TuneTarget::BACTERIA
            ? config.bacteriaParams : config.amoebaParams;

        if (IsKeyPressed(KEY_TAB))
            tuneTarget = tuneTarget == TuneTarget::BACTERIA ? TuneTarget::AMOEBA : TuneTarget::BACTERIA;

        if (IsKeyDown(KEY_Q)) adjustParam(activeParams.senseRadius,     0.8f * dt, 0.3f, 12.0f);
        if (IsKeyDown(KEY_A)) adjustParam(activeParams.senseRadius,    -0.8f * dt, 0.3f, 12.0f);
        if (IsKeyDown(KEY_W)) adjustParam(activeParams.attentionRadius, 0.5f * dt, 0.1f, 8.0f);
        if (IsKeyDown(KEY_S)) adjustParam(activeParams.attentionRadius,-0.5f * dt, 0.1f, 8.0f);
        if (IsKeyDown(KEY_E)) adjustParam(activeParams.criticalRadius,  0.3f * dt, 0.05f, 3.0f);
        if (IsKeyDown(KEY_D)) adjustParam(activeParams.criticalRadius, -0.3f * dt, 0.05f, 3.0f);
        if (IsKeyDown(KEY_R)) adjustParam(activeParams.attentionWeight,  0.4f * dt, 0.05f, 1.0f);
        if (IsKeyDown(KEY_F)) adjustParam(activeParams.attentionWeight, -0.4f * dt, 0.05f, 1.0f);

        if (IsKeyPressed(KEY_V))
        {
            if (tuneTarget == TuneTarget::BACTERIA)
                config.autoBacteriaAttention = !config.autoBacteriaAttention;
            else
                config.autoAmoebaAttention = !config.autoAmoebaAttention;
        }

        if (IsKeyPressed(KEY_Z))
        {
            if (tuneTarget == TuneTarget::BACTERIA) config.bacteriaAttention = AttentionMode::WANDER;
            else config.amoebaAttention = AttentionMode::WANDER;
        }
        if (IsKeyPressed(KEY_X))
        {
            if (tuneTarget == TuneTarget::BACTERIA) config.bacteriaAttention = AttentionMode::HUNGRY;
            else config.amoebaAttention = AttentionMode::HUNGRY;
        }
        if (IsKeyPressed(KEY_C))
        {
            if (tuneTarget == TuneTarget::BACTERIA) config.bacteriaAttention = AttentionMode::FEARFUL;
            else config.amoebaAttention = AttentionMode::FEARFUL;
        }

        if (IsKeyPressed(KEY_ONE))   { scenario = DemoScenario::BACTERIA_HEADON; setupScenario(scenario, dish, bacteria, amoeba, obstacles, flockStates, boidParams, water); stats.reset(); }
        if (IsKeyPressed(KEY_TWO))   { scenario = DemoScenario::BACTERIA_SWARM;  setupScenario(scenario, dish, bacteria, amoeba, obstacles, flockStates, boidParams, water); stats.reset(); }
        if (IsKeyPressed(KEY_THREE)) { scenario = DemoScenario::AMOEBA_NAV;      setupScenario(scenario, dish, bacteria, amoeba, obstacles, flockStates, boidParams, water); stats.reset(); }
        if (IsKeyPressed(KEY_FOUR))  { scenario = DemoScenario::COMBINED;        setupScenario(scenario, dish, bacteria, amoeba, obstacles, flockStates, boidParams, water); stats.reset(); }

        if (IsKeyPressed(KEY_SPACE))
            setupScenario(scenario, dish, bacteria, amoeba, obstacles, flockStates, boidParams, water);

        if (IsKeyPressed(KEY_G)) showDebug = !showDebug;
        if (IsKeyPressed(KEY_H)) showHelp  = !showHelp;
        if (IsKeyPressed(KEY_P)) stats.recording = !stats.recording;
        if (IsKeyPressed(KEY_O)) stats.reset();
        if (IsKeyPressed(KEY_L)) exportCsv("obstacle_experiment_log.csv", scenario, config, stats);

        if (IsKeyPressed(KEY_FIVE))
        {
            for (auto &b : bacteria) b->bsm.state.hunger = 0.85f;
        }
        if (IsKeyPressed(KEY_SIX))
        {
            for (auto &b : bacteria) b->bsm.state.fear = 0.85f;
        }

        stats.tick(dt);

        // --- Bacteria update ---
        for (size_t i = 0; i < bacteria.size(); i++)
        {
            if (flockStates.size() > i)
            {
                flockStates[i] = {bacteria[i]->getCenterOfMass(), Vector3Zero()};
                flockStates[i].alive = true;
            }

            AttentionMode mode = bacteriaAttentionFor(*bacteria[i], config);
            ObstacleSenseResult sense = senseForBacteria(*bacteria[i], obstacles, config, mode);

            if (sense.detected)
                bacteria[i]->bsm.setObstacleSense(sense.avoidDirection, sense.urgency);
            else
                bacteria[i]->bsm.setObstacleSense({0.0f, 0.0f, 0.0f}, 0.0f);

            float clearance = sense.nearestClearance;
            stats.noteBacteria(bacteria[i]->bsm.isAvoidingObstacle(), sense.urgency, clearance);

            bacteria[i]->update(dt);
            dish.applyBoundary(bacteria[i]->getNodes());
            resolveObstacleCollisions(obstacles, bacteria[i]->getNodes(), BACTERIA_BODY_PAD);
        }

        // --- Amoeba update ---
        if (amoeba)
        {
            std::vector<BoidState> emptyFlock;
            float temp = dish.temperatureAt(amoeba->getCenterPosition());
            Vector3 tempGrad = dish.temperatureGradientAt(amoeba->getCenterPosition());
            amoeba->actuate(dt, cocci, emptyFlock, temp, tempGrad, 40.0f, obstacles);
            amoeba->updatePhysicsImplicit(dt);
            dish.applyBoundary(amoeba->getNodes());
            resolveObstacleCollisions(obstacles, amoeba->getNodes(), AMOEBA_BODY_PAD);

            Vector3 pos = amoeba->getCenterPosition();
            float clearance = nearestClearance(pos, obstacles, AMOEBA_BODY_PAD);
            bool avoiding = std::string(amoeba->getStateName()).find("obstacle") != std::string::npos;
            ObstacleSenseParams p = config.amoebaParams;
            AttentionMode mode = amoebaAttentionFor(*amoeba, config);
            applySelectiveAttention(p, mode);
            ObstacleSenseResult sense = senseObstacles(
                pos, amoeba->getHeading(), obstacles, p, AMOEBA_BODY_PAD, mode);
            stats.noteAmoeba(avoiding, sense.urgency, clearance);
        }

        BeginDrawing();
        ClearBackground({5, 10, 20, 255});

        BeginScissorMode(0, 0, SCREEN_W - PANEL_W, SCREEN_H);
        BeginMode3D(camera);

        dish.draw();
        for (const auto &obs : obstacles)
            obs.draw();

        if (showDebug)
        {
            for (auto &b : bacteria)
            {
                AttentionMode mode = bacteriaAttentionFor(*b, config);
                ObstacleSenseResult sense = senseForBacteria(*b, obstacles, config, mode);
                drawSenseDebug(b->getCenterOfMass(), b->getHeading(), sense,
                               config.bacteriaParams, mode,
                               {80, 160, 255, 80}, {255, 200, 80, 100});
            }
            if (amoeba)
            {
                AttentionMode mode = amoebaAttentionFor(*amoeba, config);
                ObstacleSenseParams p = config.amoebaParams;
                applySelectiveAttention(p, mode);
                Vector3 pos = amoeba->getCenterPosition();
                ObstacleSenseResult sense = senseObstacles(
                    pos, amoeba->getHeading(), obstacles, p, AMOEBA_BODY_PAD, mode);
                drawSenseDebug(pos, amoeba->getHeading(), sense, config.amoebaParams, mode,
                               {35, 215, 170, 80}, {255, 170, 35, 100});
            }
        }

        if (amoeba) amoeba->draw(showDebug);
        for (auto &b : bacteria)
            b->draw(showDebug);

        dish.drawShell();
        EndMode3D();
        EndScissorMode();

        drawPanel(SCREEN_W - PANEL_W, 0, scenario, tuneTarget, config, stats, showDebug, showHelp);
        DrawFPS(10, SCREEN_H - 24);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
