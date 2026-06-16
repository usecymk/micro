// amoebatest.cpp
//
// Amoeba Volume / Pressure Diagram.
//
// This scene isolates the pressure-based volume-preservation model used by the
// real Amoeba (see src/Amoeba.h, actuate()) so it can be inspected on its own,
// without the hunger / prey / locomotion behaviour that normally drives the cell.
//
// The model (identical formulas + constants to Amoeba):
//
//     V_target   = (4/3)·π·r0^3                      (rest sphere)
//     V_current  = (4/3)·π·r_avg^3                    r_avg = mean |node - center|
//     P          = max(0, (V_target - V_current) · k)     k = pressureStiffness = 320
//     F_node     = (P / N) · outwardDir               applied to each membrane node
//
// Key idea to show in the talk: pressure is a ONE-SIDED restoring force — it
// only pushes outward to resist compression / fill to target; it never sucks in.

#include <vector>
#include <memory>
#include <cmath>
#include <cstdio>
#include <deque>

#include "raylib.h"
#include "raymath.h"

#include "src/PhysicsBody.h"   // Node, Spring, PhysicsBody, implicit integrator
#include "src/DragForce.h"

// ---------------------------------------------------------------------------
// AmoebaCell: a trimmed Amoeba — nucleus + Fibonacci-sphere membrane held by
// radial and neighbour springs, with the same internal-pressure volume model.
// ---------------------------------------------------------------------------
class AmoebaCell : public PhysicsBody
{
public:
    int   numMembraneNodes = 64;
    float baseRadius;
    float baseTargetVolume;     // V_target at scale = 1
    float pressureStiffness = 320.0f;

    // user / interaction state
    float volumeScale = 1.0f;   // multiplies the target volume

    // live diagnostics (refreshed by measure())
    Vector3 geomCenter   = {0, 0, 0};
    float   avgRadius    = 0.0f;
    float   targetRadius = 0.0f;
    float   currentVolume = 0.0f;
    float   targetVolume  = 0.0f;
    float   internalPressure = 0.0f;

    // The force generator that injects pressure — mirrors Amoeba::AmoebaForceInjector.
    class PressureInjector : public ForceGenerator
    {
        AmoebaCell *cell;
    public:
        explicit PressureInjector(AmoebaCell *c) : cell(c) {}
        void apply(PhysicsBody &body, float /*dt*/) override
        {
            cell->measure();
            auto &ns = body.getNodes();
            for (int i = 1; i <= cell->numMembraneNodes; i++)
            {
                Vector3 outward = Vector3Subtract(ns[i].position, cell->geomCenter);
                float len = Vector3Length(outward);
                if (len < 1e-5f) continue;
                outward = Vector3Scale(outward, 1.0f / len);
                Vector3 f = Vector3Scale(outward, cell->internalPressure / cell->numMembraneNodes);
                ns[i].force = Vector3Add(ns[i].force, f);
            }
        }
    };

    AmoebaCell(Vector3 center, float radius = 0.6667f,
               float stiffness = 12.0f, float damping = 1.8f)
    {
        baseRadius = radius;

        nodes.push_back(Node(center, 0.15f)); // nucleus (index 0)

        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t = (float)i / (numMembraneNodes - 1);
            float y = 1.0f - (t * 2.0f);
            float radius_at_y = std::sqrt(std::max(0.0f, 1.0f - y * y));
            float theta = PI * (1.0f + std::sqrt(5.0f)) * i; // golden-angle spiral

            float x = std::cos(theta) * radius_at_y;
            float z = std::sin(theta) * radius_at_y;
            Vector3 offset = Vector3Scale({x, y, z}, radius);
            nodes.push_back(Node(Vector3Add(center, offset), 0.07f));
        }

        // radial springs nucleus -> membrane
        for (int i = 1; i <= numMembraneNodes; i++)
            addSpring(0, i, stiffness * 0.6f, damping);

        // neighbour springs across the membrane surface
        float expected = std::sqrt(4.0f * PI * radius * radius / numMembraneNodes);
        float threshold = expected * 1.5f;
        for (int i = 1; i <= numMembraneNodes; i++)
            for (int j = i + 1; j <= numMembraneNodes; j++)
                if (Vector3Distance(nodes[i].position, nodes[j].position) < threshold)
                    addSpring(i, j, stiffness * 0.7f, damping);

        baseTargetVolume = (4.0f / 3.0f) * PI * std::pow(radius, 3);

        addForceGenerator(std::make_unique<PressureInjector>(this));
        addForceGenerator(std::make_unique<DragForce>(0.8f));

        measure();
    }

    // read-only diagnostics; safe to call every frame for the HUD/plot.
    void measure()
    {
        Vector3 c = Vector3Zero();
        for (int i = 1; i <= numMembraneNodes; i++)
            c = Vector3Add(c, nodes[i].position);
        geomCenter = Vector3Scale(c, 1.0f / numMembraneNodes);

        float rsum = 0.0f;
        for (int i = 1; i <= numMembraneNodes; i++)
            rsum += Vector3Distance(nodes[i].position, geomCenter);
        avgRadius = rsum / numMembraneNodes;

        currentVolume = (4.0f / 3.0f) * PI * std::pow(avgRadius, 3);
        targetVolume  = baseTargetVolume * volumeScale;
        targetRadius  = std::cbrt(targetVolume / ((4.0f / 3.0f) * PI));

        float delta = targetVolume - currentVolume;
        internalPressure = std::max(0.0f, delta * pressureStiffness);
    }

    // external disturbances for the demo (applied directly to velocities)
    void squeeze(float gain, float dt)
    {
        for (int i = 1; i <= numMembraneNodes; i++)
        {
            Vector3 inward = Vector3Subtract(geomCenter, nodes[i].position);
            nodes[i].velocity = Vector3Add(nodes[i].velocity, Vector3Scale(inward, gain * dt));
        }
    }

    void poke(float magnitude)
    {
        for (auto &n : nodes)
        {
            Vector3 r = { (float)GetRandomValue(-100, 100),
                          (float)GetRandomValue(-100, 100),
                          (float)GetRandomValue(-100, 100) };
            float l = Vector3Length(r);
            if (l > 1e-4f)
                n.velocity = Vector3Add(n.velocity, Vector3Scale(r, magnitude / l));
        }
    }

    void resetShape()
    {
        int idx = 0;
        nodes[0].position = geomCenter;
        nodes[0].velocity = Vector3Zero();
        Vector3 c = geomCenter;
        for (int i = 0; i < numMembraneNodes; i++)
        {
            float t = (float)i / (numMembraneNodes - 1);
            float y = 1.0f - (t * 2.0f);
            float radius_at_y = std::sqrt(std::max(0.0f, 1.0f - y * y));
            float theta = PI * (1.0f + std::sqrt(5.0f)) * i;
            Vector3 offset = Vector3Scale({std::cos(theta) * radius_at_y, y,
                                           std::sin(theta) * radius_at_y}, baseRadius);
            nodes[i + 1].position = Vector3Add(c, offset);
            nodes[i + 1].velocity = Vector3Zero();
        }
        (void)idx;
        measure();
    }
};

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
static void drawArrow(Vector3 from, Vector3 dir, float len, Color c)
{
    if (len < 1e-4f) return;
    Vector3 to = Vector3Add(from, Vector3Scale(dir, len));
    DrawLine3D(from, to, c);
    DrawSphere(to, 0.02f, c);
}

int main()
{
    AmoebaCell cell({0.0f, 0.0f, 0.0f});

    InitWindow(1280, 720, "micro3d — Amoeba Volume / Pressure Diagram");
    SetTargetFPS(60);

    // ── manual orbit camera (keeps the keyboard free for the demo) ───────
    Camera3D camera = {
        {2.6f, 1.4f, 2.6f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        45.0f, CAMERA_PERSPECTIVE
    };
    float yaw = 0.78f, pitch = 0.5f, dist = 3.4f;

    bool paused     = false;
    bool showArrows = true;

    std::deque<float> volHist;   // current/target volume ratio over time
    std::deque<float> presHist;  // pressure over time
    const int HIST = 260;

    while (!WindowShouldClose())
    {
        float dt = std::min(GetFrameTime(), 1.0f / 30.0f);

        // ── camera control (left-drag orbit, wheel zoom) ────────────────
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 d = GetMouseDelta();
            yaw   -= d.x * 0.005f;
            pitch += d.y * 0.005f;
            pitch  = Clamp(pitch, -1.4f, 1.4f);
        }
        dist -= GetMouseWheelMove() * 0.3f;
        dist  = Clamp(dist, 1.6f, 8.0f);
        camera.position = {
            dist * std::cos(pitch) * std::cos(yaw),
            dist * std::sin(pitch),
            dist * std::cos(pitch) * std::sin(yaw)
        };

        // ── input ────────────────────────────────────────────────────────
        if (IsKeyDown(KEY_UP))   cell.volumeScale = Clamp(cell.volumeScale + 0.5f * dt, 0.4f, 1.8f);
        if (IsKeyDown(KEY_DOWN)) cell.volumeScale = Clamp(cell.volumeScale - 0.5f * dt, 0.4f, 1.8f);
        if (IsKeyDown(KEY_K))    cell.squeeze(6.0f, dt);   // compress (lower current volume)
        if (IsKeyDown(KEY_L))    cell.squeeze(-3.5f, dt);  // pull outward
        if (IsKeyPressed(KEY_SPACE)) cell.poke(2.2f);
        if (IsKeyPressed(KEY_F)) showArrows = !showArrows;
        if (IsKeyPressed(KEY_R)) { cell.volumeScale = 1.0f; cell.resetShape(); }
        if (IsKeyPressed(KEY_ENTER)) paused = !paused;

        // ── step ───────────────────────────────────────────────────────
        if (!paused)
            cell.updatePhysicsImplicit(dt);
        cell.measure();

        if (volHist.size() >= (size_t)HIST) { volHist.pop_front(); presHist.pop_front(); }
        volHist.push_back(cell.currentVolume / std::max(cell.targetVolume, 1e-4f));
        presHist.push_back(cell.internalPressure);

        // ── draw 3D ──────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground({8, 12, 20, 255});
        BeginMode3D(camera);

        auto &nodes   = cell.getNodes();
        auto &springs = cell.getSprings();

        // target-volume reference sphere (where the cell "wants" to be)
        DrawSphereWires(cell.geomCenter, cell.targetRadius, 12, 12, {90, 230, 150, 90});

        // membrane mesh (membrane-membrane springs only)
        for (auto &s : springs)
        {
            int a = (int)(s.nodeA - &nodes[0]);
            int b = (int)(s.nodeB - &nodes[0]);
            if (a == 0 || b == 0) continue; // skip radial spokes for clarity
            DrawLine3D(s.nodeA->position, s.nodeB->position, {70, 150, 175, 120});
        }

        // membrane nodes
        for (int i = 1; i <= cell.numMembraneNodes; i++)
            DrawSphere(nodes[i].position, 0.022f, {120, 230, 235, 255});

        // nucleus + geometric center
        DrawSphere(nodes[0].position, 0.06f, {255, 210, 120, 255});
        DrawSphere(cell.geomCenter, 0.04f, {255, 255, 255, 200});

        // outward pressure-force arrows (length ∝ per-node pressure)
        if (showArrows)
        {
            float perNode = cell.internalPressure / cell.numMembraneNodes;
            for (int i = 1; i <= cell.numMembraneNodes; i++)
            {
                Vector3 outward = Vector3Subtract(nodes[i].position, cell.geomCenter);
                float l = Vector3Length(outward);
                if (l < 1e-5f) continue;
                outward = Vector3Scale(outward, 1.0f / l);
                drawArrow(nodes[i].position, outward,
                          std::min(perNode * 1.6f, 0.8f), {255, 150, 70, 230});
            }
        }

        EndMode3D();

        // ── HUD: numbers ──────────────────────────────────────────────────
        DrawRectangle(10, 10, 360, 232, {6, 14, 24, 210});
        DrawRectangleLines(10, 10, 360, 232, {95, 170, 210, 180});
        DrawText("Amoeba — Volume / Pressure", 24, 22, 20, RAYWHITE);

        float deltaV = cell.targetVolume - cell.currentVolume;
        DrawText(TextFormat("Target volume   V_t = %.3f", cell.targetVolume), 24, 56, 16, {150, 230, 170, 255});
        DrawText(TextFormat("Current volume  V   = %.3f", cell.currentVolume), 24, 76, 16, {150, 220, 235, 255});
        DrawText(TextFormat("Delta V         = %+.3f", deltaV), 24, 96, 16, {220, 220, 220, 255});
        DrawText(TextFormat("avg radius %.3f   target %.3f", cell.avgRadius, cell.targetRadius),
                 24, 116, 14, {180, 200, 215, 255});

        DrawText(TextFormat("Pressure  P = %.2f", cell.internalPressure), 24, 142, 18, {255, 170, 90, 255});
        DrawText(TextFormat("P = max(0, (V_t - V) * k),  k = %.0f", cell.pressureStiffness),
                 24, 166, 14, {200, 180, 150, 255});
        DrawText(TextFormat("Membrane nodes N = %d", cell.numMembraneNodes), 24, 188, 14, {180, 200, 215, 255});
        DrawText(deltaV > 0.0f ? "state: inflating  (pressure pushing out)"
                               : "state: at/over target  (P = 0, springs only)",
                 24, 212, 14, deltaV > 0.0f ? Color{255, 180, 110, 255} : Color{150, 200, 160, 255});

        // ── P–V diagram (right panel) ──────────────────────────────────────
        const int px = 905, py = 60, pw = 350, ph = 250;
        DrawRectangle(px - 14, py - 26, pw + 28, ph + 70, {6, 14, 24, 210});
        DrawRectangleLines(px - 14, py - 26, pw + 28, ph + 70, {95, 170, 210, 180});
        DrawText("Pressure  vs  Volume", px, py - 20, 16, RAYWHITE);

        float Vmin = 0.0f, Vmax = 2.0f * cell.baseTargetVolume;
        float Pmax = 0.55f * cell.pressureStiffness * cell.baseTargetVolume;

        auto sx = [&](float V) { return px + (V - Vmin) / (Vmax - Vmin) * pw; };
        auto sy = [&](float P) { return py + ph - Clamp(P / Pmax, 0.0f, 1.0f) * ph; };

        DrawRectangleLines(px, py, pw, ph, {80, 110, 130, 200});
        // axis labels
        DrawText("V", px + pw - 12, py + ph + 6, 14, {170, 190, 205, 255});
        DrawText("P", px - 12, py - 2, 14, {170, 190, 205, 255});

        // restoring law P = k·max(0, V_t - V)
        for (int i = 0; i < pw; i++)
        {
            float V0 = Vmin + (Vmax - Vmin) * ((float)i / pw);
            float V1 = Vmin + (Vmax - Vmin) * ((float)(i + 1) / pw);
            float P0 = std::max(0.0f, (cell.targetVolume - V0) * cell.pressureStiffness);
            float P1 = std::max(0.0f, (cell.targetVolume - V1) * cell.pressureStiffness);
            DrawLine((int)sx(V0), (int)sy(P0), (int)sx(V1), (int)sy(P1), {255, 160, 80, 220});
        }

        // target-volume line (equilibrium, P = 0)
        DrawLine((int)sx(cell.targetVolume), py, (int)sx(cell.targetVolume), py + ph, {90, 230, 150, 160});
        DrawText("V_t", (int)sx(cell.targetVolume) + 3, py + 4, 12, {120, 230, 150, 220});

        // current operating point
        float cxp = sx(cell.currentVolume);
        float cyp = sy(cell.internalPressure);
        DrawCircle((int)cxp, (int)cyp, 6.0f, {120, 220, 235, 255});
        DrawCircleLines((int)cxp, (int)cyp, 6.0f, RAYWHITE);
        DrawLine((int)cxp, py + ph, (int)cxp, (int)cyp, {120, 220, 235, 90});

        // ── time-series strip (volume ratio) ──────────────────────────────
        const int tx = 24, ty = 470, tw = 350, th = 150;
        DrawRectangle(tx - 14, ty - 26, tw + 28, th + 52, {6, 14, 24, 210});
        DrawRectangleLines(tx - 14, ty - 26, tw + 28, th + 52, {95, 170, 210, 180});
        DrawText("Volume ratio  V / V_t  over time", tx, ty - 20, 16, RAYWHITE);
        DrawRectangleLines(tx, ty, tw, th, {80, 110, 130, 200});
        // ratio = 1 line
        int oneY = ty + th - (int)(((1.0f - 0.4f) / (1.6f - 0.4f)) * th);
        DrawLine(tx, oneY, tx + tw, oneY, {90, 230, 150, 150});
        DrawText("1.0", tx + tw + 2, oneY - 6, 12, {120, 230, 150, 200});
        for (size_t i = 1; i < volHist.size(); i++)
        {
            float r0 = Clamp(volHist[i - 1], 0.4f, 1.6f);
            float r1 = Clamp(volHist[i],     0.4f, 1.6f);
            float x0 = tx + tw * ((float)(i - 1) / HIST);
            float x1 = tx + tw * ((float)i / HIST);
            float y0 = ty + th - ((r0 - 0.4f) / (1.6f - 0.4f)) * th;
            float y1 = ty + th - ((r1 - 0.4f) / (1.6f - 0.4f)) * th;
            DrawLine((int)x0, (int)y0, (int)x1, (int)y1, {120, 220, 235, 255});
        }

        // controls
        DrawText("UP/DOWN target volume   K squeeze   L expand   SPACE poke   R reset   ENTER pause   F arrows",
                 14, 690, 15, {175, 195, 210, 255});
        DrawText("left-drag = orbit   wheel = zoom", 14, 670, 14, {140, 165, 185, 255});
        if (paused) DrawText("PAUSED", 1180, 14, 20, {255, 200, 120, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
