#pragma once
#include <vector>
#include <cmath>
#include <cstdlib>
#include "raylib.h"
#include "raymath.h"

// NOTE: PhysicsBody (and Node/Spring) must be defined in the including .cpp
//       before this header is included — same pattern as before.

// ================================================================
//  Motor Controllers
//  (checklist §3.3 — swim-MC, left-MC, right-MC)
// ================================================================

struct SwimMC
{
    float speed = 0.5f;     // 0 = still, 1 = full throttle

    float getAmplitude() const { return 0.03f + speed * 0.08f; }  // scaled 1/5
    float getFrequency() const { return 1.5f  + speed * 3.0f;  }
    // Thrust is wave-activity coupled (amplitude × frequency × speed),
    // so stopping the wave stops propulsion — no magic hidden force.
    float getThrust()    const { return speed * getAmplitude() * getFrequency() * 2.0f; }
};

struct TurnMC
{
    float left  = 0.0f;    // 0–1 activation → steer left
    float right = 0.0f;    // 0–1 activation → steer right
    float pitch = 0.0f;    // –1…+1 → pitch down / up
};

// ================================================================
//  Internal State
//  (checklist §6.1 — hunger H, fear F)
// ================================================================

struct InternalState
{
    // ── vitals ────────────────────────────────────────────────────
    float hunger = 0.0f;   // 0 = full, 1 = starving
    float fear   = 0.0f;   // 0 = calm, 1 = terrified

    // ── death ─────────────────────────────────────────────────────
    enum class CauseOfDeath { NONE, STARVATION, ATTACK };
    bool        alive        = true;
    CauseOfDeath causeOfDeath = CauseOfDeath::NONE;

    // ── attack tracking (3 hits within 7 s = death) ───────────────
    int   hitCount       = 0;
    float hitWindowTimer = 0.0f;   // counts down; resets to 7 s on first hit

    void update(float dt)
    {
        if (!alive) return;

        hunger += 0.015f * dt;
        hunger  = Clamp(hunger, 0.0f, 1.0f);
        fear   *= expf(-1.2f * dt);

        // Slide the attack window forward
        if (hitWindowTimer > 0.0f) {
            hitWindowTimer -= dt;
            if (hitWindowTimer <= 0.0f) hitCount = 0;   // window expired, reset
        }

        // Starvation death
        if (hunger >= 1.0f) {
            alive        = false;
            causeOfDeath = CauseOfDeath::STARVATION;
        }
    }

    void onEat() { hunger = 0.0f; }

    void onPredatorNearby(float proximity)   // proximity 0=far, 1=touching
    {
        fear = Clamp(fear + proximity * 0.6f, 0.0f, 1.0f);
    }

    // Call when a predator/organism actually strikes (not just proximity)
    void onAttackHit()
    {
        if (hitWindowTimer <= 0.0f) {       // fresh window
            hitCount       = 1;
            hitWindowTimer = 7.0f;
        } else {
            hitCount++;
        }
        onPredatorNearby(0.8f);             // also spike fear

        if (hitCount >= 3) {
            alive        = false;
            causeOfDeath = CauseOfDeath::ATTACK;
        }
    }
};

// ================================================================
//  Behavior states
//  (checklist §5 — intention generator priority chain)
// ================================================================

enum class Behavior { WANDER, SEEK_FOOD, ESCAPE };

// ================================================================
//  Bacterium
// ================================================================

class Bacterium : public PhysicsBody
{
    // ── layout constants ──────────────────────────────────────────
    static const int BODY_NODES  = 4;
    static const int FLAG_NODES  = 14;
    static const int TOTAL_NODES = BODY_NODES + FLAG_NODES;

    // ── physics internals ─────────────────────────────────────────
    float time = 0.0f;
    float drag = 0.97f;

    // ── orientation (derived from physics each frame) ─────────────
    Vector3 heading    = { 0.0f, 0.0f, -1.0f };
    float   currentYaw = 0.0f;

    // ── wander state ──────────────────────────────────────────────
    float wanderTimer = 0.0f;
    float targetYaw   = 0.0f;

    // ── escape state ──────────────────────────────────────────────
    // Set by onWallHit() / onPredatorNearby() — direction to flee TOWARD
    Vector3 fleeDirection   = { 0.0f, 0.0f, 0.0f };
    float   wallHitCooldown = 0.0f;  // seconds until onWallHit() may snap targetYaw again

public:
    // ── public interface ──────────────────────────────────────────
    SwimMC        swimMC;
    TurnMC        turnMC;
    InternalState state;
    Behavior      behavior = Behavior::WANDER;

    // spawnPos: world-space centre the bacterium appears at
    Bacterium(Vector3 spawnPos = { 0.0f, 2.0f, 0.0f },
              float stiffness  = 60.0f,
              float damping    = 2.0f)
    {
        float sx = spawnPos.x, sy = spawnPos.y, sz = spawnPos.z;
        float segLen = 0.0045f; // 20 × 0.0045 m ≈ 0.09 m total flagellum

        nodes.reserve(TOTAL_NODES);

        // body nodes – stiff pill along local Z (1/5 scale: ±0.08, ±0.026)
        nodes.push_back(Node({ sx, sy, sz - 0.080f }));  // 0 front
        nodes.push_back(Node({ sx, sy, sz - 0.026f }));  // 1
        nodes.push_back(Node({ sx, sy, sz + 0.026f }));  // 2
        nodes.push_back(Node({ sx, sy, sz + 0.080f }));  // 3 back

        // flagellum nodes – extend in +Z from node 3 (body tail at sz + 0.080)
        for (int i = 0; i < FLAG_NODES; i++)
        {
            float z = sz + 0.080f + (i + 1) * segLen;
            nodes.push_back(Node({ sx, sy, z }));
        }

        float k = stiffness, d = damping;

        // chain springs – maintain body length (stay stiff)
        addSpring(0, 1, k,          d);
        addSpring(1, 2, k,          d);
        addSpring(2, 3, k,          d);
        // short cross-braces – moderate stiffness keeps pill width, allows slight shear
        addSpring(0, 2, k * 0.15f,  d);
        addSpring(1, 3, k * 0.15f,  d);
        // long diagonal – very soft so the body can curve gently when steering
        addSpring(0, 3, k * 0.03f,  d);

        // flagellum – tapered stiffness: stiffer at base, softer toward tip.
        // Springs must be stiff enough to resist the wave force (k*restLen > 2*F).
        for (int i = 0; i < FLAG_NODES; i++)
        {
            float t    = (float)i / (float)(FLAG_NODES - 1);  // 0 = base, 1 = tip
            float fk_i = k * (0.25f - t * 0.15f);  // base: 15.0, tip: 6.0
            float fd_i = d * (0.25f - t * 0.18f);  // base:  0.5, tip: 0.14
            addSpring(BODY_NODES - 1 + i, BODY_NODES + i, fk_i, fd_i);
        }

        // seed orientation
        initOrientation();
    }

    // ── accessors ─────────────────────────────────────────────────

    Vector3 getHeading() const { return heading; }

    // Call this whenever something is hit — wall now, organisms later.
    // awayDir: unit vector pointing AWAY from the thing that was hit.
    // intensity: 0–1, how scary the collision is.
    void onWallHit(Vector3 awayDir, float intensity = 0.6f)
    {
        state.onPredatorNearby(intensity);
        fleeDirection = Vector3Normalize(awayDir);
        // Only snap targetYaw on the FIRST contact per cooldown window.
        // Without this guard the target was re-locked every frame, which
        // prevented the organism from ever completing its escape turn.
        if (wallHitCooldown <= 0.0f)
        {
            targetYaw       = atan2f(fleeDirection.x, fleeDirection.z);
            wallHitCooldown = 1.5f;
        }
    }

    // Tear down and rebuild in-place — use for respawn after death.
    void reset(Vector3 spawnPos = { 0.0f, 2.0f, 0.0f })
    {
        nodes.clear();
        springs.clear();

        time            = 0.0f;
        heading         = { 0.0f, 0.0f, -1.0f };
        currentYaw      = 0.0f;
        wanderTimer     = 0.0f;
        targetYaw       = 0.0f;
        fleeDirection   = { 0.0f, 0.0f, 0.0f };
        wallHitCooldown = 0.0f;
        state           = InternalState{};
        behavior        = Behavior::WANDER;
        swimMC          = SwimMC{};
        turnMC          = TurnMC{};

        float sx = spawnPos.x, sy = spawnPos.y, sz = spawnPos.z;
        float segLen = 0.0045f;
        float k = 60.0f, d = 2.0f;
        nodes.reserve(TOTAL_NODES);
        nodes.push_back(Node({ sx, sy, sz - 0.080f }));
        nodes.push_back(Node({ sx, sy, sz - 0.026f }));
        nodes.push_back(Node({ sx, sy, sz + 0.026f }));
        nodes.push_back(Node({ sx, sy, sz + 0.080f }));
        for (int i = 0; i < FLAG_NODES; i++)
            nodes.push_back(Node({ sx, sy, sz + 0.080f + (i + 1) * segLen }));

        addSpring(0, 1, k,          d);
        addSpring(1, 2, k,          d);
        addSpring(2, 3, k,          d);
        addSpring(0, 2, k * 0.15f,  d);
        addSpring(1, 3, k * 0.15f,  d);
        addSpring(0, 3, k * 0.03f,  d);
        for (int i = 0; i < FLAG_NODES; i++)
        {
            float t    = (float)i / (float)(FLAG_NODES - 1);
            float fk_i = k * (0.25f - t * 0.15f);  // base: 15.0, tip: 6.0
            float fd_i = d * (0.25f - t * 0.18f);  // base:  0.5, tip: 0.14
            addSpring(BODY_NODES - 1 + i, BODY_NODES + i, fk_i, fd_i);
        }

        initOrientation();
    }

    Vector3 getCenterOfMass()
    {
        Vector3 com = Vector3Zero();
        for (int i = 0; i < BODY_NODES; i++)
            com = Vector3Add(com, nodes[i].position);
        return Vector3Scale(com, 1.0f / BODY_NODES);
    }

    // ── main update ───────────────────────────────────────────────

    void update(float dt)
    {
        for (auto& n : nodes) n.force = Vector3Zero();
        updateHeading();
        state.update(dt);
        if (wallHitCooldown > 0.0f) wallHitCooldown -= dt;
        updateBehavior(dt);
        applyMotorControls();
        animateFlagellum(dt);
        updatePhysicsWithDrag(dt);
    }

    // ── draw  (pass debugOverlay=true to show heading + state) ────

    void draw(bool debugOverlay = false)
    {
        if (!state.alive) return;   // dead → vanish entirely

        auto& ns = getNodes();

        // pill body (1/5 scale radius)
        const float radius  = 0.036f;
        const Color bodyCol = { 80, 220, 120, 255 };
        for (int i = 0; i < BODY_NODES - 1; i++)
            DrawCapsule(ns[i].position, ns[i+1].position, radius, 6, 4, bodyCol);
        DrawSphere(ns[0].position, radius, bodyCol);  // front cap only; rear uses capsule end

        // blue flagellum – tapers toward tip (1/5 scale thickness)
        for (int i = BODY_NODES - 1; i < TOTAL_NODES - 1; i++)
        {
            float t         = (float)(i - (BODY_NODES - 1)) / (float)FLAG_NODES;
            float thickness = 0.011f * (1.0f - t * 0.75f);
            Color col       = { 100, 210, 255, 255 };
            DrawCapsule(ns[i].position, ns[i+1].position, thickness, 4, 2, col);
        }

        // ── debug overlays ────────────────────────────────────────
        if (debugOverlay)
        {
            Vector3 com = getCenterOfMass();

            // heading arrow (yellow) — scaled to match organism size
            DrawLine3D(com, Vector3Add(com, Vector3Scale(heading, 0.2f)), YELLOW);
            DrawSphere(Vector3Add(com, Vector3Scale(heading, 0.2f)), 0.012f, YELLOW);

            // right vector (red)
            Vector3 worldUp    = { 0.0f, 1.0f, 0.0f };
            Vector3 localRight = Vector3Normalize(
                Vector3CrossProduct(heading, worldUp));
            DrawLine3D(com, Vector3Add(com, Vector3Scale(localRight, 0.1f)), RED);
        }
    }

private:
    // ── orientation ───────────────────────────────────────────────

    void initOrientation()
    {
        Vector3 bf = Vector3Subtract(nodes[0].position, nodes[BODY_NODES-1].position);
        if (Vector3Length(bf) > 1e-4f)
        {
            heading    = Vector3Normalize(bf);
            currentYaw = atan2f(heading.x, heading.z);
        }
        targetYaw = currentYaw;
    }

    void updateHeading()
    {
        Vector3 bf = Vector3Subtract(nodes[0].position, nodes[BODY_NODES-1].position);
        if (Vector3Length(bf) > 1e-4f)
        {
            heading    = Vector3Normalize(bf);
            currentYaw = atan2f(heading.x, heading.z);
        }
    }

    // ── behavior FSM ──────────────────────────────────────────────
    //  Priority chain (checklist Fig. 12):
    //    escape > seek food > wander

    void updateBehavior(float dt)
    {
        if      (state.fear   > 0.5f) behavior = Behavior::ESCAPE;
        else if (state.hunger > 0.7f) behavior = Behavior::SEEK_FOOD;
        else                          behavior = Behavior::WANDER;

        switch (behavior)
        {
            case Behavior::WANDER:    doWander(dt);    break;
            case Behavior::SEEK_FOOD: doSeekFood(dt);  break;
            case Behavior::ESCAPE:    doEscape(dt);    break;
        }
    }

    // Wander – random heading changes at random intervals
    void doWander(float dt)
    {
        swimMC.speed = 0.4f;
        wanderTimer -= dt;
        if (wanderTimer <= 0.0f)
        {
            // next turn happens in 2–5 seconds
            wanderTimer = 2.0f + (float)(rand() % 300) / 100.0f;
            float randTurn = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * 1.2f;
            targetYaw = currentYaw + randTurn;
        }
        steerTowardYaw();
    }

    // Seek food – placeholder: swim faster straight until food particles exist
    void doSeekFood(float /*dt*/)
    {
        swimMC.speed   = 0.75f;
        turnMC.left    = 0.0f;
        turnMC.right   = 0.0f;
    }

    // Escape – max speed, steer away from whatever triggered the fear.
    // fleeDirection and targetYaw are set by onWallHit() (walls now,
    // predators later — same call, same response).
    void doEscape(float /*dt*/)
    {
        swimMC.speed = 1.0f;
        if (Vector3Length(fleeDirection) > 0.1f)
            steerTowardYaw();   // targetYaw was already set by onWallHit()
    }

    // Smoothly steer the heading toward targetYaw
    void steerTowardYaw()
    {
        float diff = targetYaw - currentYaw;
        while (diff >  3.14159265f) diff -= 2.0f * 3.14159265f;
        while (diff < -3.14159265f) diff += 2.0f * 3.14159265f;

        const float deadzone = 0.08f;
        if (diff > deadzone)
        {
            turnMC.left  = fminf(1.0f, diff * 1.5f);
            turnMC.right = 0.0f;
        }
        else if (diff < -deadzone)
        {
            turnMC.right = fminf(1.0f, -diff * 1.5f);
            turnMC.left  = 0.0f;
        }
        else
        {
            turnMC.left = turnMC.right = 0.0f;
        }
    }

    // ── motor control forces ──────────────────────────────────────

    void applyMotorControls()
    {
        // During ESCAPE, blend thrust toward the flee direction so the organism
        // can break away from a wall even before it has fully turned.
        // Weight 2× flee vs 1× heading so it definitely moves away.
        Vector3 thrustDir = heading;
        if (behavior == Behavior::ESCAPE && Vector3Length(fleeDirection) > 0.1f)
        {
            Vector3 blended = Vector3Add(heading, Vector3Scale(fleeDirection, 2.0f));
            if (Vector3Length(blended) > 1e-4f)
                thrustDir = Vector3Normalize(blended);
        }

        // Swim MC: wave-activity-coupled thrust (no flagella → no force).
        float thrust = swimMC.getThrust();
        for (int i = 0; i < BODY_NODES; i++)
            nodes[i].force = Vector3Add(nodes[i].force,
                                        Vector3Scale(thrustDir, thrust));

        // Turn MC: small lateral nudge only on the two FRONT body nodes.
        // With the long diagonal spring now very soft, this creates a visible
        // gentle curve in the body rather than a rigid rotation.
        Vector3 worldUp    = { 0.0f, 1.0f, 0.0f };
        Vector3 localRight = Vector3Normalize(
            Vector3CrossProduct(heading, worldUp));

        float net = turnMC.right - turnMC.left;   // + = turn right
        nodes[0].force = Vector3Add(nodes[0].force,
            Vector3Scale(localRight, net * 0.2f));  // front node (1/5 scale)
        nodes[1].force = Vector3Add(nodes[1].force,
            Vector3Scale(localRight, net * 0.06f)); // 2nd node   (1/5 scale)

        // Pitch MC: gentle vertical nudge on front node
        nodes[0].force.y += turnMC.pitch * 0.18f;  // (1/5 scale)
    }

    // ── flagellum animation ───────────────────────────────────────
    //  Beat direction is kept perpendicular to the current body axis
    //  so it looks correct as the organism turns.

    void animateFlagellum(float dt)
    {
        time += dt;

        // local side = perpendicular to body axis, in horizontal plane
        Vector3 bf = Vector3Subtract(nodes[BODY_NODES-1].position, nodes[0].position);
        if (Vector3Length(bf) < 1e-4f) return;
        Vector3 bodyAxis = Vector3Normalize(bf);
        Vector3 worldUp  = { 0.0f, 1.0f, 0.0f };
        Vector3 side     = Vector3Normalize(Vector3CrossProduct(bodyAxis, worldUp));

        float curAmp  = swimMC.getAmplitude();
        float curFreq = swimMC.getFrequency();
        float phaseStep = 0.45f;   // 14 nodes × 0.45 rad ≈ 1 full wave cycle visible

        for (int i = 0; i < FLAG_NODES; i++)
        {
            float t   = (float)i / (float)(FLAG_NODES - 1);  // 0 = base, 1 = tip
            float amp = curAmp * (1.0f + t * 0.4f);          // slight grow toward tip
            float fMag = 2.0f * amp * sinf(curFreq * time - i * phaseStep);

            // Force tapers down toward tip so tip doesn't whip freely
            float forceScale = 1.0f - t * 0.55f;
            nodes[BODY_NODES + i].force = Vector3Add(
                nodes[BODY_NODES + i].force,
                Vector3Scale(side, fMag * forceScale));
        }
    }

    // ── physics step (spring forces + drag integration) ───────────

    void updatePhysicsWithDrag(float dt)
    {
        for (auto& s : springs)
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L   = Vector3Length(d);
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
            n.acceleration = Vector3Scale(n.force, 1.0f / (float)n.mass);
            n.velocity     = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.velocity     = Vector3Scale(n.velocity, drag);   // fluid drag
            n.position     = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        }
    }
};
