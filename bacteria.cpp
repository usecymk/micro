// bacteria.cpp
// ============
// Main entry point for the bacteria simulation demo.
// Replaces testing.cpp — compile with buildbacteria.sh
//
// Key bindings:
//   C      — cycle camera (Free → Follow → Overhead)
//   D      — toggle debug overlay (heading, state bars, behavior)
//   F      — trigger fear  (for testing the FSM)
//   E      — feed bacterium (resets hunger)
//   SPACE  — pause / unpause
//   ESC    — quit

#include <vector>
#include <cmath>
#include "raylib.h"
#include "raymath.h"

// ── Physics base (kept here; partner has their own copy in main.cpp) ──────────

const Vector3 GLOBAL_GRAVITY = { 0.0f, -9.81f, 0.0f };

struct Node
{
    Vector3 position, velocity, force, acceleration;
    double  mass;
    Node(Vector3 pos, float m = 1.0f)
        : position(pos), velocity(Vector3Zero()), force(Vector3Zero()),
          acceleration(Vector3Zero()), mass(m) {}
};

struct Spring
{
    Node* nodeA, *nodeB;
    float rest_length, stiffness, damping;
    Spring(Node* A, Node* B, float rl = 1.0f, float s = 1.0f, float d = 1.0f)
        : nodeA(A), nodeB(B), rest_length(rl), stiffness(s), damping(d) {}
};

class PhysicsBody
{
protected:
    std::vector<Node>   nodes;
    std::vector<Spring> springs;

    void addSpring(int a, int b, float k, float d)
    {
        Vector3 pa = nodes[a].position, pb = nodes[b].position;
        float dx = pb.x - pa.x, dy = pb.y - pa.y, dz = pb.z - pa.z;
        float rest = std::sqrt(dx*dx + dy*dy + dz*dz);
        springs.emplace_back(&nodes[a], &nodes[b], rest, k, d);
    }

public:
    std::vector<Node>&   getNodes()   { return nodes;   }
    std::vector<Spring>& getSprings() { return springs; }

    void updatePhysicsExplicit(float dt)
    {
        for (auto& n : nodes) n.force = Vector3Zero();
        for (auto& s : springs)
        {
            Vector3 d    = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L      = Vector3Length(d);
            if (L < 1e-6f) continue;
            Vector3 dhat = Vector3Normalize(d);
            float fs     = s.stiffness * (L - s.rest_length);
            float vrel   = Vector3DotProduct(
                Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), dhat);
            float fd     = s.damping * vrel;
            Vector3 F    = Vector3Scale(dhat, -(fs + fd));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add   (s.nodeB->force, F);
        }
        for (auto& n : nodes)
        {
            n.force        = Vector3Add(n.force, Vector3Scale(GLOBAL_GRAVITY, n.mass));
            n.acceleration = Vector3Scale(n.force, 1.0f / (float)n.mass);
            n.velocity     = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.position     = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        }
    }
};

// ── Our components ────────────────────────────────────────────────────────────
#include "bacteria.h"
#include "petri.h"

// ── Camera mode ───────────────────────────────────────────────────────────────

enum class ViewMode { FREE, FOLLOW, OVERHEAD, POV };

static const char* camLabel(ViewMode m)
{
    switch (m)
    {
        case ViewMode::FREE:     return "FREE CAM  (WASD + mouse)";
        case ViewMode::FOLLOW:   return "FOLLOW CAM";
        case ViewMode::OVERHEAD: return "OVERHEAD CAM";
        case ViewMode::POV:      return "BACTERIUM POV";
    }
    return "";
}

// ── HUD helpers ───────────────────────────────────────────────────────────────

static void DrawBar(int x, int y, int w, int h,
                    float value, Color fill, const char* label)
{
    DrawText(label, x, y, 14, LIGHTGRAY);
    DrawRectangle(x, y + 16, w, h, { 40, 40, 40, 200 });
    DrawRectangle(x, y + 16, (int)(w * value), h, fill);
    DrawRectangleLines(x, y + 16, w, h, GRAY);
}

static const char* behaviorLabel(Behavior b)
{
    switch (b)
    {
        case Behavior::WANDER:    return "WANDER";
        case Behavior::SEEK_FOOD: return "SEEK FOOD";
        case Behavior::ESCAPE:    return "ESCAPE";
    }
    return "?";
}

// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    InitWindow(1280, 720, "Micro-Life 3D — Bacteria Demo");
    SetTargetFPS(60);
    DisableCursor();

    // ── Camera ────────────────────────────────────────────────────
    Camera3D camera     = {};
    camera.position     = { 6.0f, 4.0f, 0.5f };
    camera.target       = { 0.0f, 2.0f, 0.0f };
    camera.up           = { 0.0f, 1.0f, 0.0f };
    camera.fovy         = 45.0f;
    camera.projection   = CAMERA_PERSPECTIVE;
    ViewMode camMode  = ViewMode::FREE;

    // ── Spawn ─────────────────────────────────────────────────────
    Bacterium bact({ 0.0f, 2.0f, 0.0f });

    bool showDebug = false;
    bool paused    = false;

    // ─────────────────────────────────────────────────────────────
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // ── Input ─────────────────────────────────────────────────
        if (IsKeyPressed(KEY_C))
        {
            switch (camMode)
            {
                case ViewMode::FREE:     camMode = ViewMode::FOLLOW;   break;
                case ViewMode::FOLLOW:   camMode = ViewMode::OVERHEAD; break;
                case ViewMode::OVERHEAD: camMode = ViewMode::POV;      break;
                case ViewMode::POV:      camMode = ViewMode::FREE;     break;
            }
        }
        if (IsKeyPressed(KEY_D))     showDebug = !showDebug;
        if (IsKeyPressed(KEY_SPACE)) paused    = !paused;

        // FSM test keys
        if (IsKeyPressed(KEY_F))  bact.state.onAttackHit();              // simulate hit  → ESCAPE / death
        if (IsKeyPressed(KEY_E))  bact.state.onEat();                   // feed it        → clear hunger
        if (IsKeyPressed(KEY_H))  bact.state.hunger = 0.8f;             // starve it      → SEEK FOOD
        if (IsKeyPressed(KEY_R) && !bact.state.alive)                   // respawn after death
            bact.reset({ 0.0f, 2.0f, 0.0f });

        // ── Simulation ────────────────────────────────────────────
        if (!paused && bact.state.alive)
        {
            bact.update(dt);
            ApplyDishBoundaryAll(bact.getNodes());  // physical bounce only, no fear
        }

        // ── Camera update ─────────────────────────────────────────
        Vector3 com = bact.getCenterOfMass();

        switch (camMode)
        {
            case ViewMode::FREE:
                UpdateCamera(&camera, CAMERA_FREE);
                break;

            case ViewMode::FOLLOW:
            {
                // Smoothly track from behind and slightly above
                Vector3 head   = bact.getHeading();
                Vector3 behind = Vector3Subtract(com, Vector3Scale(head, 3.5f));
                behind.y      += 1.5f;
                camera.position = Vector3Lerp(camera.position, behind, 6.0f * dt);
                camera.target   = Vector3Lerp(camera.target,   com,    6.0f * dt);
                break;
            }

            case ViewMode::OVERHEAD:
                camera.position = { com.x, com.y + 9.0f, com.z + 0.01f };
                camera.target   = com;
                camera.fovy     = 45.0f;
                break;

            case ViewMode::POV:
            {
                // Mount the camera just ahead of the front body node,
                // looking in the heading direction — bacterium's eye view.
                Vector3 front = bact.getNodes()[0].position;
                Vector3 head  = bact.getHeading();
                // Offset slightly forward so the pill body isn't in frame
                camera.position = Vector3Add(front, Vector3Scale(head, 0.06f));
                camera.target   = Vector3Add(front, Vector3Scale(head, 1.0f));
                camera.up       = { 0.0f, 1.0f, 0.0f };
                camera.fovy     = 90.0f;   // wide-angle — fish-eye feel
                break;
            }
        }

        // ── Draw ──────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground({ 5, 10, 20, 255 });

        BeginMode3D(camera);
            DrawPetriDish();
            bact.draw(showDebug);
        EndMode3D();

        // ── HUD ───────────────────────────────────────────────────
        DrawFPS(10, 10);

        // Camera mode label
        DrawText(camLabel(camMode), 10, 30, 16, SKYBLUE);

        // Key hints
        DrawText("[C] cam  [D] debug  [SPACE] pause  [F] attack  [H] hunger  [E] eat",
                 10, 52, 13, { 120, 120, 120, 255 });

        if (paused)
            DrawText("-- PAUSED --", GetScreenWidth()/2 - 60, 12, 22, ORANGE);

        // Debug panel
        if (showDebug)
        {
            int px = 10, py = 90;

            DrawBar(px, py,      160, 12, bact.state.hunger, ORANGE, "HUNGER");
            DrawBar(px, py + 40, 160, 12, bact.state.fear,   RED,    "FEAR");

            const char* bhv = behaviorLabel(bact.behavior);
            DrawText(TextFormat("BEHAVIOR : %s", bhv), px, py + 80, 14, WHITE);
            DrawText(TextFormat("SWIM SPD : %.0f%%", bact.swimMC.speed * 100.0f),
                     px, py + 100, 14, GREEN);
            DrawText(TextFormat("TURN L/R : %.2f / %.2f",
                                 bact.turnMC.left, bact.turnMC.right),
                     px, py + 120, 14, LIME);
            DrawText(TextFormat("HITS     : %d / 3  (%.1fs left)",
                                 bact.state.hitCount, bact.state.hitWindowTimer),
                     px, py + 140, 14, ORANGE);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
