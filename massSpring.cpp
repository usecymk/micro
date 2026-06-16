// massSpring.cpp
//
// A standalone scene that illustrates the micro3d PhysicsEngine architecture:
//
//     Node (point mass)  ──spring──  Node      <- the deformable substrate
//          ▲                                       (PhysicsBody)
//          │  net force
//      ForceGenerator(s)  <- gravity / buoyancy / drag / impulse  (Strategy)
//          │
//      Integrator         <- implicit (backward Euler) or semi-implicit Euler
//
// Everything here is built from the SAME primitives the organisms use
// (Node, Spring, PhysicsBody, ForceGenerator), just arranged into a clean
// 3D lattice so each piece of the engine is visible in isolation.

#include <vector>
#include <memory>
#include <cmath>
#include <cstdio>

#include "raylib.h"
#include "raymath.h"

#include "src/PhysicsBody.h"        // Node, Spring, PhysicsBody, integrators
#include "src/FluidEnvironment.h"
#include "src/GravityForce.h"
#include "src/BuoyancyForce.h"
#include "src/DragForce.h"

// ---------------------------------------------------------------------------
// ToggleForce: a thin decorator around any ForceGenerator that lets the scene
// switch a strategy on/off at runtime. This is the whole point of the
// ForceGenerator interface — forces are composable, pluggable objects.
// ---------------------------------------------------------------------------
class ToggleForce : public ForceGenerator
{
public:
    bool        enabled;
    const char *name;
    std::unique_ptr<ForceGenerator> inner;

    ToggleForce(const char *n, std::unique_ptr<ForceGenerator> f, bool on = true)
        : enabled(on), name(n), inner(std::move(f)) {}

    void apply(PhysicsBody &body, float dt) override
    {
        if (enabled && inner)
            inner->apply(body, dt);
    }
};

// ---------------------------------------------------------------------------
// PokeForce: a transient force generator. When triggered it injects a brief
// directional impulse into every node, then fades out. Demonstrates that a
// force generator can carry its own state and lifetime.
// ---------------------------------------------------------------------------
class PokeForce : public ForceGenerator
{
public:
    float   timer    = 0.0f;
    float   strength = 90.0f;
    Vector3 dir      = {0.0f, 1.0f, 0.0f};

    void trigger(Vector3 d, float duration = 0.12f)
    {
        dir   = d;
        timer = duration;
    }

    void apply(PhysicsBody &body, float dt) override
    {
        if (timer <= 0.0f)
            return;
        timer -= dt;
        for (auto &n : body.getNodes())
            n.force = Vector3Add(n.force, Vector3Scale(dir, strength * n.mass));
    }
};

// ---------------------------------------------------------------------------
// LatticeBody: a PhysicsBody arranged as a 3D grid of nodes wired together
// with structural, shear and bend springs — a textbook mass-spring lattice.
// ---------------------------------------------------------------------------
class LatticeBody : public PhysicsBody
{
public:
    int     nx, ny, nz;
    float   spacing;
    Vector3 origin;
    bool    pinned = true;

    std::vector<int>     pinnedIdx;
    std::vector<Vector3> anchors;

    LatticeBody(Vector3 org,
                int NX = 4, int NY = 4, int NZ = 4, float sp = 0.6f,
                float kStruct = 130.0f, float kShear = 55.0f, float kBend = 30.0f,
                float damp = 2.2f)
        : nx(NX), ny(NY), nz(NZ), spacing(sp), origin(org)
    {
        build(kStruct, kShear, kBend, damp);
    }

    int idx(int i, int j, int k) const { return (i * ny + j) * nz + k; }

    Vector3 gridPos(int i, int j, int k) const
    {
        return { origin.x + i * spacing,
                 origin.y + j * spacing,
                 origin.z + k * spacing };
    }

    Vector3 center() const
    {
        return { origin.x + (nx - 1) * 0.5f * spacing,
                 origin.y + (ny - 1) * 0.5f * spacing,
                 origin.z + (nz - 1) * 0.5f * spacing };
    }

    void build(float kStruct, float kShear, float kBend, float damp)
    {
        nodes.clear();
        springs.clear();
        nodes.reserve(nx * ny * nz);

        for (int i = 0; i < nx; i++)
            for (int j = 0; j < ny; j++)
                for (int k = 0; k < nz; k++)
                    nodes.push_back(Node(gridPos(i, j, k))); // default mass 1, volume 1e-3

        auto link = [&](int ai, int aj, int ak, int bi, int bj, int bk, float k, float d)
        {
            if (bi < 0 || bi >= nx || bj < 0 || bj >= ny || bk < 0 || bk >= nz)
                return;
            addSpring(idx(ai, aj, ak), idx(bi, bj, bk), k, d);
        };

        // structural springs: nearest neighbours along each axis
        for (int i = 0; i < nx; i++)
            for (int j = 0; j < ny; j++)
                for (int k = 0; k < nz; k++)
                {
                    link(i, j, k, i + 1, j, k, kStruct, damp);
                    link(i, j, k, i, j + 1, k, kStruct, damp);
                    link(i, j, k, i, j, k + 1, kStruct, damp);
                }

        // shear (face diagonals) + bend (body diagonals) per unit cell
        for (int i = 0; i < nx - 1; i++)
            for (int j = 0; j < ny - 1; j++)
                for (int k = 0; k < nz - 1; k++)
                {
                    // xy-plane face diagonals
                    link(i, j, k, i + 1, j + 1, k, kShear, damp);
                    link(i + 1, j, k, i, j + 1, k, kShear, damp);
                    // xz-plane face diagonals
                    link(i, j, k, i + 1, j, k + 1, kShear, damp);
                    link(i + 1, j, k, i, j, k + 1, kShear, damp);
                    // yz-plane face diagonals
                    link(i, j, k, i, j + 1, k + 1, kShear, damp);
                    link(i, j + 1, k, i, j, k + 1, kShear, damp);
                    // body (space) diagonals
                    link(i, j, k, i + 1, j + 1, k + 1, kBend, damp);
                    link(i + 1, j, k, i, j + 1, k + 1, kBend, damp);
                    link(i, j + 1, k, i + 1, j, k + 1, kBend, damp);
                    link(i, j, k + 1, i + 1, j + 1, k, kBend, damp);
                }

        // pin the entire top layer (j == ny - 1) so the lattice hangs
        capturePins();
    }

    void capturePins()
    {
        pinnedIdx.clear();
        anchors.clear();
        for (int i = 0; i < nx; i++)
            for (int k = 0; k < nz; k++)
            {
                int id = idx(i, ny - 1, k);
                pinnedIdx.push_back(id);
                anchors.push_back(nodes[id].position);
            }
    }

    bool isPinned(int nodeIndex) const
    {
        if (!pinned)
            return false;
        for (int id : pinnedIdx)
            if (id == nodeIndex)
                return true;
        return false;
    }

    // Re-project pinned nodes back onto their anchors after the solve.
    void enforcePins()
    {
        if (!pinned)
            return;
        for (size_t p = 0; p < pinnedIdx.size(); p++)
        {
            Node &n   = nodes[pinnedIdx[p]];
            n.position = anchors[p];
            n.velocity = Vector3Zero();
        }
    }

    // A simple floor at y = floorY (the base integrators clamp at y = -5,
    // this gives us a visible floor that matches the rendered grid).
    void clampFloor(float floorY, float restitution = 0.35f)
    {
        for (auto &n : nodes)
        {
            if (n.position.y < floorY)
            {
                n.position.y = floorY;
                if (n.velocity.y < 0.0f)
                    n.velocity.y = -n.velocity.y * restitution;
            }
        }
    }

    void resetState()
    {
        for (int i = 0; i < nx; i++)
            for (int j = 0; j < ny; j++)
                for (int k = 0; k < nz; k++)
                {
                    Node &n    = nodes[idx(i, j, k)];
                    n.position = gridPos(i, j, k);
                    n.velocity = Vector3Zero();
                    n.force    = Vector3Zero();
                }
    }
};

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------
static Color lerpColor(Color a, Color b, float t)
{
    t = Clamp(t, 0.0f, 1.0f);
    return {
        (unsigned char)(a.r + (b.r - a.r) * t),
        (unsigned char)(a.g + (b.g - a.g) * t),
        (unsigned char)(a.b + (b.b - a.b) * t),
        (unsigned char)(a.a + (b.a - a.a) * t),
    };
}

// colour a spring by its strain: blue = compressed, teal = rest, red = stretched
static Color strainColor(float strain)
{
    const Color compressed = { 90, 150, 255, 220};
    const Color rest       = { 70, 220, 200, 220};
    const Color stretched  = {255, 110,  90, 220};
    float s = Clamp(strain / 0.35f, -1.0f, 1.0f);
    if (s >= 0.0f) return lerpColor(rest, stretched, s);
    return lerpColor(rest, compressed, -s);
}

static void drawArrow(Vector3 from, Vector3 vec, float scale, Color c)
{
    float mag = Vector3Length(vec);
    if (mag < 1e-4f)
        return;
    float len = std::min(mag * scale, 1.4f);
    Vector3 dir = Vector3Scale(vec, 1.0f / mag);
    Vector3 to  = Vector3Add(from, Vector3Scale(dir, len));
    DrawLine3D(from, to, c);
    DrawSphere(to, 0.035f, c);
}

int main()
{
    // ── world / fluid ────────────────────────────────────────────────────
    const float floorY   = 0.0f;
    FluidEnvironment water;
    water.density  = 1000.0f;
    water.surfaceY = 3.4f;                 // lattice starts fully submerged
    water.gravity  = {0.0f, -9.81f, 0.0f};

    // ── the lattice (a PhysicsBody) ──────────────────────────────────────
    LatticeBody lattice({-0.9f, 1.0f, -0.9f}, 4, 4, 4, 0.6f);

    // ── force generators (the Strategy objects we plug in) ───────────────
    auto gravityTog = std::make_unique<ToggleForce>(
        "GravityForce", std::make_unique<GravityForce>(water.gravity), true);
    auto buoyTog = std::make_unique<ToggleForce>(
        "BuoyancyForce", std::make_unique<BuoyancyForce>(&water), false);
    auto dragTog = std::make_unique<ToggleForce>(
        "DragForce", std::make_unique<DragForce>(0.8f), true);
    auto pokeFg = std::make_unique<PokeForce>();

    ToggleForce *gravity = gravityTog.get();
    ToggleForce *buoyancy = buoyTog.get();
    ToggleForce *drag = dragTog.get();
    PokeForce   *poke = pokeFg.get();

    lattice.addForceGenerator(std::move(gravityTog));
    lattice.addForceGenerator(std::move(buoyTog));
    lattice.addForceGenerator(std::move(dragTog));
    lattice.addForceGenerator(std::move(pokeFg));

    // ── window / camera ──────────────────────────────────────────────────
    InitWindow(1280, 720, "micro3d — PhysicsEngine Architecture: Node-Spring Lattice");

    Vector3 center = lattice.center();
    Camera3D camera = {
        Vector3Add(center, {4.2f, 1.6f, 4.2f}),
        center,
        {0.0f, 1.0f, 0.0f},
        45.0f,
        CAMERA_PERSPECTIVE
    };

    bool useImplicit = true;
    bool showArrows  = true;
    bool showSprings = true;
    bool paused      = false;

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = std::min(GetFrameTime(), 1.0f / 30.0f); // clamp to keep solve sane

        // ── input ──────────────────────────────────────────────────────
        if (IsKeyPressed(KEY_G)) gravity->enabled  = !gravity->enabled;
        if (IsKeyPressed(KEY_B)) buoyancy->enabled = !buoyancy->enabled;
        if (IsKeyPressed(KEY_C)) drag->enabled     = !drag->enabled;   // C = "drag/Cushion"
        if (IsKeyPressed(KEY_I)) useImplicit        = !useImplicit;
        if (IsKeyPressed(KEY_F)) showArrows         = !showArrows;
        if (IsKeyPressed(KEY_V)) showSprings        = !showSprings;
        if (IsKeyPressed(KEY_P)) lattice.pinned     = !lattice.pinned;
        if (IsKeyPressed(KEY_SPACE))
        {
            float a = (float)GetRandomValue(0, 360) * DEG2RAD;
            poke->trigger(Vector3Normalize({std::cos(a), 1.6f, std::sin(a)}));
        }
        if (IsKeyPressed(KEY_R))
        {
            lattice.resetState();
            lattice.pinned = true;
        }
        if (IsKeyPressed(KEY_ENTER)) paused = !paused;

        // ── step the engine ─────────────────────────────────────────────
        if (!paused)
        {
            if (useImplicit) lattice.updatePhysicsImplicit(dt);
            else             lattice.updatePhysicsExplicit(dt);
            lattice.enforcePins();
            lattice.clampFloor(floorY);
        }

        // ── draw ─────────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground({8, 12, 22, 255});
        BeginMode3D(camera);

        DrawGrid(24, 0.5f);

        // water surface (only meaningful when buoyancy is active)
        if (buoyancy->enabled)
        {
            float s = 7.0f;
            Vector3 c = lattice.center();
            DrawTriangle3D({c.x - s, water.surfaceY, c.z - s},
                           {c.x - s, water.surfaceY, c.z + s},
                           {c.x + s, water.surfaceY, c.z + s}, {40, 110, 170, 60});
            DrawTriangle3D({c.x - s, water.surfaceY, c.z - s},
                           {c.x + s, water.surfaceY, c.z + s},
                           {c.x + s, water.surfaceY, c.z - s}, {40, 110, 170, 60});
        }

        auto &nodes   = lattice.getNodes();
        auto &springs = lattice.getSprings();

        // springs, coloured by strain
        if (showSprings)
        {
            for (auto &sp : springs)
            {
                float L = Vector3Distance(sp.nodeA->position, sp.nodeB->position);
                float strain = (sp.rest_length > 1e-5f)
                                   ? (L - sp.rest_length) / sp.rest_length
                                   : 0.0f;
                DrawLine3D(sp.nodeA->position, sp.nodeB->position, strainColor(strain));
            }
        }

        // nodes (point masses); pinned ones highlighted
        for (size_t i = 0; i < nodes.size(); i++)
        {
            bool pin = lattice.isPinned((int)i);
            Color c  = pin ? Color{255, 90, 90, 255} : Color{120, 230, 235, 255};
            DrawSphere(nodes[i].position, pin ? 0.075f : 0.06f, c);
        }

        // net force arrows on free nodes
        if (showArrows)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (lattice.isPinned((int)i))
                    continue;
                drawArrow(nodes[i].position, nodes[i].force, 0.03f, {255, 220, 120, 230});
            }
        }

        EndMode3D();

        // ── HUD: architecture mapping ──────────────────────────────────
        DrawRectangle(10, 10, 360, 250, {6, 14, 24, 210});
        DrawRectangleLines(10, 10, 360, 250, {95, 170, 210, 180});
        DrawText("PhysicsEngine architecture", 24, 22, 20, RAYWHITE);

        DrawText(TextFormat("Nodes (point masses):  %d", (int)nodes.size()),
                 24, 54, 16, {200, 225, 235, 255});
        DrawText(TextFormat("Springs (damped Hooke): %d", (int)springs.size()),
                 24, 74, 16, {200, 225, 235, 255});
        DrawText(TextFormat("Integrator: %s",
                            useImplicit ? "Implicit (backward Euler)"
                                        : "Semi-implicit Euler"),
                 24, 94, 16, {255, 230, 160, 255});

        DrawText("ForceGenerators (Strategy):", 24, 122, 16, {180, 210, 255, 255});
        auto fgLine = [](int y, const char *key, const char *label, bool on)
        {
            Color c = on ? Color{120, 230, 150, 255} : Color{120, 130, 145, 255};
            DrawText(TextFormat("[%s] %s %s", key, on ? "x" : " ", label), 36, y, 15, c);
        };
        fgLine(144, "G", "GravityForce", gravity->enabled);
        fgLine(162, "B", "BuoyancyForce", buoyancy->enabled);
        fgLine(180, "C", "DragForce", drag->enabled);
        fgLine(198, "Sp", "PokeForce (impulse)", poke->timer > 0.0f);

        DrawText(TextFormat("Top layer pinned: %s", lattice.pinned ? "yes" : "no"),
                 24, 222, 15, {200, 225, 235, 255});

        // controls
        DrawText("G/B/C force gens   I integrator   P pin   SPACE poke   R reset   ENTER pause",
                 14, 678, 16, {180, 200, 215, 255});
        DrawText("F arrows   V springs   |   WASD + mouse = camera",
                 14, 698, 16, {150, 175, 195, 255});

        if (paused)
            DrawText("PAUSED", 1180, 14, 20, {255, 200, 120, 255});

        DrawFPS(1180, 690);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
