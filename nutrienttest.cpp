// nutrienttest.cpp
//
// Nutrient field visualization demo.
//
// Showcases how the diffuse nutrient field (src/Nutrient.h) is modelled and
// rendered. The field itself is a sum of Gaussian "blob" sources:
//
//     c(x) = Σ_i  strength_i · exp( -|x - center_i|^2 / (2 σ_i^2) )
//
// and organisms sense it through concentrationAt() / gradientAt(). The shimmer
// you see in the main sim is ~1000+ billboard particles advected through that
// field for visualization only — they do not affect the simulation.
//
// This demo layers four views over the SAME field so each piece is legible:
//   [P] particle rendering   (the real NutrientField::draw)
//   [H] concentration heatmap slice at an adjustable height
//   [G] analytic gradient arrows (the chemotaxis directions)
//   [C] volumetric concentration point cloud (the continuous field on a grid)
// plus a movable probe that prints c(x) and its gradient at a point.

#include <cmath>
#include <cstdio>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "src/Nutrient.h"
#include "src/PetriDish.h"

// number of sources / particles we ask the field to build (kept here for HUD;
// NutrientField doesn't expose its internal counts).
static constexpr int   FIELD_BLOBS     = 6;
static constexpr int   FIELD_PARTICLES = 1300;

static Color lerpC(Color a, Color b, float t)
{
    t = Clamp(t, 0.0f, 1.0f);
    return { (unsigned char)(a.r + (b.r - a.r) * t),
             (unsigned char)(a.g + (b.g - a.g) * t),
             (unsigned char)(a.b + (b.b - a.b) * t),
             (unsigned char)(a.a + (b.a - a.a) * t) };
}

// concentration -> colour ramp for the diagram overlays (0..1)
static Color ramp(float t, unsigned char alpha)
{
    t = Clamp(t, 0.0f, 1.0f);
    Color c;
    if (t < 0.33f)      c = lerpC({10, 22, 60, 255},  {25, 130, 175, 255}, t / 0.33f);
    else if (t < 0.66f) c = lerpC({25, 130, 175, 255}, {45, 215, 150, 255}, (t - 0.33f) / 0.33f);
    else                c = lerpC({45, 215, 150, 255}, {255, 255, 220, 255}, (t - 0.66f) / 0.34f);
    c.a = alpha;
    return c;
}

static void drawArrow3D(Vector3 from, Vector3 dir, float len, Color c)
{
    if (len < 1e-4f) return;
    Vector3 to = Vector3Add(from, Vector3Scale(dir, len));
    DrawLine3D(from, to, c);
    DrawSphere(to, 0.06f, c);
}

// horizontal concentration slice rendered as translucent quads
static void drawHeatSlice(const NutrientField &field, float y, float radius, int res)
{
    float maxC = field.maxConcentration();
    float cell = (2.0f * radius) / res;

    BeginBlendMode(BLEND_ALPHA);
    rlDisableDepthMask();
    rlBegin(RL_QUADS);
    for (int i = 0; i < res; i++)
    {
        for (int j = 0; j < res; j++)
        {
            float x = -radius + (i + 0.5f) * cell;
            float z = -radius + (j + 0.5f) * cell;
            if (x * x + z * z > radius * radius) continue;

            float t = field.concentrationAt({x, y, z}) / maxC;
            if (t < 0.02f) continue;
            unsigned char a = (unsigned char)(Clamp(t * 1.4f, 0.0f, 1.0f) * 190.0f);
            Color col = ramp(t, a);

            float x0 = -radius + i * cell, x1 = x0 + cell;
            float z0 = -radius + j * cell, z1 = z0 + cell;
            rlColor4ub(col.r, col.g, col.b, col.a);
            rlVertex3f(x0, y, z0);
            rlVertex3f(x0, y, z1);
            rlVertex3f(x1, y, z1);
            rlVertex3f(x1, y, z0);
        }
    }
    rlEnd();
    rlEnableDepthMask();
    EndBlendMode();
}

// coarse grid of analytic gradient arrows in the slice plane
static void drawGradientField(const NutrientField &field, float y, float radius, int res)
{
    float maxC = field.maxConcentration();
    float step = (2.0f * radius) / res;
    for (int i = 0; i <= res; i++)
        for (int j = 0; j <= res; j++)
        {
            float x = -radius + i * step;
            float z = -radius + j * step;
            if (x * x + z * z > radius * radius) continue;

            Vector3 p = {x, y, z};
            Vector3 g = field.gradientAt(p);
            float mag = Vector3Length(g);
            if (mag < 1e-4f) continue;

            Vector3 dir = Vector3Scale(g, 1.0f / mag);
            float t = field.concentrationAt(p) / maxC;
            float len = std::min(0.25f + mag * 0.9f, step * 0.95f);
            drawArrow3D(p, dir, len, ramp(Clamp(t * 1.3f, 0.0f, 1.0f), 235));
        }
}

// volumetric point cloud of the continuous field
static void drawConcentrationCloud(const NutrientField &field, float radius,
                                   float floorY, float ceilY, float step)
{
    float maxC = field.maxConcentration();
    for (float x = -radius; x <= radius; x += step)
        for (float z = -radius; z <= radius; z += step)
        {
            if (x * x + z * z > radius * radius) continue;
            for (float y = floorY; y <= ceilY; y += step)
            {
                float t = field.concentrationAt({x, y, z}) / maxC;
                if (t < 0.12f) continue;
                Color c = ramp(t, (unsigned char)(Clamp(t, 0.0f, 1.0f) * 220.0f));
                DrawPoint3D({x, y, z}, c);
            }
        }
}

int main()
{
    PetriDish dish(6.0f, 5.0f, 0.0f);

    NutrientField field;
    field.init(dish.radius, dish.floorY, dish.ceilY(), FIELD_PARTICLES, FIELD_BLOBS);

    InitWindow(1280, 720, "micro3d — Nutrient Field Visualization");
    SetTargetFPS(60);

    // orbit camera (mouse), keyboard free for the controls
    Vector3 target = {0.0f, dish.ceilY() * 0.5f, 0.0f};
    Camera3D camera = { {0, 0, 0}, target, {0, 1, 0}, 45.0f, CAMERA_PERSPECTIVE };
    float yaw = 0.8f, pitch = 0.45f, distCam = 15.0f;

    bool showParticles = true;
    bool showHeat      = true;
    bool showGradient  = false;
    bool showCloud     = false;
    bool paused        = false;

    float sliceY = dish.floorY + dish.height * 0.5f;
    Vector3 probe = {0.0f, sliceY, 0.0f};

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // camera
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 d = GetMouseDelta();
            yaw   -= d.x * 0.005f;
            pitch += d.y * 0.005f;
            pitch  = Clamp(pitch, -1.4f, 1.4f);
        }
        distCam -= GetMouseWheelMove() * 0.6f;
        distCam  = Clamp(distCam, 4.0f, 30.0f);
        camera.position = Vector3Add(target, {
            distCam * std::cos(pitch) * std::cos(yaw),
            distCam * std::sin(pitch),
            distCam * std::cos(pitch) * std::sin(yaw) });

        // toggles
        if (IsKeyPressed(KEY_P)) showParticles = !showParticles;
        if (IsKeyPressed(KEY_H)) showHeat      = !showHeat;
        if (IsKeyPressed(KEY_G)) showGradient  = !showGradient;
        if (IsKeyPressed(KEY_C)) showCloud     = !showCloud;
        if (IsKeyPressed(KEY_ENTER)) paused    = !paused;
        if (IsKeyPressed(KEY_R))
            field.init(dish.radius, dish.floorY, dish.ceilY(), FIELD_PARTICLES, FIELD_BLOBS);

        // slice height
        if (IsKeyDown(KEY_LEFT_BRACKET))  sliceY = Clamp(sliceY - 2.0f * dt, dish.floorY, dish.ceilY());
        if (IsKeyDown(KEY_RIGHT_BRACKET)) sliceY = Clamp(sliceY + 2.0f * dt, dish.floorY, dish.ceilY());

        // probe movement (XZ), clamped to dish
        float pspeed = 4.0f * dt;
        if (IsKeyDown(KEY_LEFT))  probe.x -= pspeed;
        if (IsKeyDown(KEY_RIGHT)) probe.x += pspeed;
        if (IsKeyDown(KEY_UP))    probe.z -= pspeed;
        if (IsKeyDown(KEY_DOWN))  probe.z += pspeed;
        probe.y = sliceY;
        float pr = std::sqrt(probe.x * probe.x + probe.z * probe.z);
        if (pr > dish.radius) { probe.x *= dish.radius / pr; probe.z *= dish.radius / pr; }

        if (!paused)
            field.update(dt);

        // ── draw ───────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground({5, 9, 16, 255});
        BeginMode3D(camera);

        DrawGrid(24, 0.5f);

        if (showCloud)
            drawConcentrationCloud(field, dish.radius, dish.floorY, dish.ceilY(), 0.7f);

        if (showHeat)
            drawHeatSlice(field, sliceY, dish.radius, 56);

        if (showGradient)
            drawGradientField(field, sliceY, dish.radius, 11);

        if (showParticles)
            field.draw(camera);

        // probe
        float maxC      = field.maxConcentration();
        float probeConc = field.concentrationAt(probe);
        Vector3 probeGrad = field.gradientAt(probe);
        DrawSphere(probe, 0.12f, RAYWHITE);
        DrawSphereWires(probe, 0.18f, 6, 6, {255, 255, 255, 120});
        {
            float gm = Vector3Length(probeGrad);
            if (gm > 1e-4f)
                drawArrow3D(probe, Vector3Scale(probeGrad, 1.0f / gm),
                            std::min(0.6f + gm, 2.2f), {255, 220, 120, 255});
        }

        dish.drawShell();
        EndMode3D();

        // ── HUD: model explanation ─────────────────────────────────────
        DrawRectangle(10, 10, 430, 210, {6, 14, 24, 210});
        DrawRectangleLines(10, 10, 430, 210, {95, 170, 210, 180});
        DrawText("Nutrient field visualization", 24, 22, 20, RAYWHITE);
        DrawText("c(x) = SUM strength * exp(-|x-center|^2 / 2*sigma^2)",
                 24, 52, 15, {150, 220, 200, 255});
        DrawText(TextFormat("Gaussian sources (blobs): %d", FIELD_BLOBS), 24, 76, 15, {200, 225, 235, 255});
        DrawText(TextFormat("Render particles (advected): %d", FIELD_PARTICLES), 24, 96, 15, {200, 225, 235, 255});
        DrawText(TextFormat("max concentration: %.3f", maxC), 24, 116, 15, {200, 225, 235, 255});
        DrawText(TextFormat("slice height y = %.2f  (of %.1f)", sliceY, dish.ceilY()), 24, 136, 15, {255, 230, 160, 255});
        DrawText("Particles are visualization only; agents read c(x) & grad c(x).",
                 24, 162, 14, {160, 185, 205, 255});
        DrawText(TextFormat("blobs regenerate after grazing (regen relaxation)"),
                 24, 184, 14, {160, 185, 205, 255});

        // ── HUD: probe readout ─────────────────────────────────────────
        DrawRectangle(10, 232, 430, 96, {6, 14, 24, 210});
        DrawRectangleLines(10, 232, 430, 96, {95, 170, 210, 180});
        DrawText("Probe", 24, 242, 18, RAYWHITE);
        DrawText(TextFormat("pos  (%.2f, %.2f, %.2f)", probe.x, probe.y, probe.z),
                 24, 268, 15, {200, 225, 235, 255});
        DrawText(TextFormat("c(x) = %.4f   ( %.0f%% of max )",
                            probeConc, 100.0f * probeConc / maxC),
                 24, 288, 15, {150, 220, 200, 255});
        DrawText(TextFormat("|grad c| = %.4f  -> chemotaxis direction (arrow)",
                            Vector3Length(probeGrad)),
                 24, 308, 14, {255, 220, 120, 255});

        // legend (color ramp)
        int lx = 1080, ly = 30, lw = 170, lh = 16;
        DrawText("low", lx - 30, ly, 14, {150, 175, 195, 255});
        DrawText("high", lx + lw + 4, ly, 14, {150, 175, 195, 255});
        for (int i = 0; i < lw; i++)
        {
            Color c = ramp((float)i / lw, 255);
            DrawRectangle(lx + i, ly, 1, lh, c);
        }
        DrawText("concentration", lx, ly + lh + 4, 14, {180, 200, 215, 255});

        // toggles state + controls
        auto tog = [](int x, int y, const char *k, const char *label, bool on) {
            Color c = on ? Color{120, 230, 150, 255} : Color{120, 130, 145, 255};
            DrawText(TextFormat("[%s] %s %s", k, on ? "x" : " ", label), x, y, 15, c);
        };
        tog(1010, 70,  "P", "particles", showParticles);
        tog(1010, 90,  "H", "heat slice", showHeat);
        tog(1010, 110, "G", "gradient",   showGradient);
        tog(1010, 130, "C", "cloud",      showCloud);

        DrawText("[ / ] slice height   arrows = move probe   R reseed   ENTER pause",
                 14, 690, 15, {175, 195, 210, 255});
        DrawText("left-drag = orbit   wheel = zoom", 14, 670, 14, {140, 165, 185, 255});
        if (paused) DrawText("PAUSED", 1180, 14, 20, {255, 200, 120, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
