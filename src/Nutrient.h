#ifndef MICRO3D_NUTRIENT_H
#define MICRO3D_NUTRIENT_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

class NutrientField
{
public:
    struct Blob
    {
        Vector3 center;
        float   sigma;         // spatial spread of this nutrient source
        float   strength;      // current peak concentration contribution
        float   baseStrength;  // value the source regenerates back toward
    };

    struct Particle
    {
        Vector3 position;
        Vector3 velocity;   
        float   size;  
        float   conc;
        float   age;
        float   lifespan;
    };

    // ---- lifecycle ----
    void init(float dishRadius, float floorY, float ceilY,
              int particleCount = 1400, int blobCount = 7)
    {
        radius_ = dishRadius;
        floorY_ = floorY;
        ceilY_  = ceilY;


        blobs_.clear();
        float strongestBlob = 1e-4f;
        for (int i = 0; i < blobCount; i++)
        {
            Blob b;
            b.center       = randomPointInDish(1.4f);
            b.sigma        = 1.4f + randf() * 3.0f;
            b.baseStrength = 0.45f + randf() * 0.9f;
            b.strength     = b.baseStrength;
            blobs_.push_back(b);
            strongestBlob  = std::max(strongestBlob, b.baseStrength);
        }
        maxConc_ = strongestBlob;   // realistic peak achievable at a single point

        particles_.assign((size_t)std::max(0, particleCount), Particle{});
        for (auto &p : particles_)
        {
            respawn(p);
            p.age = randf() * p.lifespan;
        }
    }

    float concentrationAt(Vector3 pos) const
    {
        float c = 0.0f;
        for (const auto &b : blobs_)
        {
            float s2 = std::max(b.sigma * b.sigma, 1e-4f);
            Vector3 d = Vector3Subtract(pos, b.center);
            float dist2 = Vector3DotProduct(d, d);
            c += b.strength * std::exp(-dist2 / (2.0f * s2));
        }
        return c;
    }

    float maxConcentration() const { return maxConc_; }

    // Returns a unit vector toward the blob with the highest score (strength / dist²).
    // Stable across frames — doesn't spin as the bacterium moves between blobs.
    Vector3 bestFoodDirection(Vector3 pos) const
    {
        float   bestScore = -1.0f;
        Vector3 bestDir   = {0.0f, 0.0f, 0.0f};
        for (const auto &b : blobs_)
        {
            Vector3 toBlob = Vector3Subtract(b.center, pos);
            float   dist   = Vector3Length(toBlob);
            if (dist < 1e-4f) continue;
            float score = b.strength / (1.0f + dist * dist);
            if (score > bestScore)
            {
                bestScore = score;
                bestDir   = Vector3Scale(toBlob, 1.0f / dist);
            }
        }
        return bestDir;
    }

    Vector3 gradientAt(Vector3 pos) const
    {
        Vector3 g = Vector3Zero();
        for (const auto &b : blobs_)
        {
            float s2 = std::max(b.sigma * b.sigma, 1e-4f);
            Vector3 toCenter = Vector3Subtract(b.center, pos);
            float dist2 = Vector3DotProduct(toCenter, toCenter);
            float w = b.strength * std::exp(-dist2 / (2.0f * s2));
            g = Vector3Add(g, Vector3Scale(toCenter, w / s2));
        }
        return g;
    }


    float feedAt(Vector3 pos, float bite = 0.45f)
    {
        float available = concentrationAt(pos);
        if (available <= 1e-3f) return 0.0f;

        for (auto &b : blobs_)
        {
            float s2 = std::max(b.sigma * b.sigma, 1e-4f);
            Vector3 d = Vector3Subtract(pos, b.center);
            float w = std::exp(-Vector3DotProduct(d, d) / (2.0f * s2)); // 0..1
            b.strength = std::max(0.0f, b.strength - bite * w);
        }
        return available;
    }

    void update(float dt)
    {

        float regen = std::min(1.0f, regenRate_ * dt);
        for (auto &b : blobs_)
            b.strength += (b.baseStrength - b.strength) * regen;

        for (auto &p : particles_)
        {
            p.age += dt;

            // gentle drift + a little jitter so the field shimmers
            p.velocity.x += (randf() - 0.5f) * 0.6f * dt;
            p.velocity.y += (randf() - 0.5f) * 0.6f * dt;
            p.velocity.z += (randf() - 0.5f) * 0.6f * dt;
            p.velocity = Vector3Scale(p.velocity, 0.96f); // viscous damping

            p.position = Vector3Add(p.position, Vector3Scale(p.velocity, dt));
            reflectIntoDish(p.position, p.velocity);

            p.conc = concentrationAt(p.position) / maxConc_;

            if (p.age >= p.lifespan)
                respawn(p);
        }
    }

    void draw(const Camera3D &camera) const
    {
        if (particles_.empty()) return;

        Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
        Vector3 right   = Vector3Normalize(Vector3CrossProduct(forward, camera.up));
        Vector3 up      = Vector3CrossProduct(right, forward);

        BeginBlendMode(BLEND_ADDITIVE);
        rlDisableDepthMask();

        rlBegin(RL_QUADS);
        for (const auto &p : particles_)
        {
            float fade = lifeFade(p);
            float a    = std::min(1.0f, p.conc * 1.6f) * fade;
            if (a <= 0.01f) continue;

            Color col = concentrationColor(p.conc, a);

            float s = p.size * (0.6f + 0.8f * p.conc);
            Vector3 rx = Vector3Scale(right, s);
            Vector3 uy = Vector3Scale(up, s);

            Vector3 c0 = Vector3Subtract(Vector3Subtract(p.position, rx), uy);
            Vector3 c1 = Vector3Subtract(Vector3Add(p.position, rx), uy);
            Vector3 c2 = Vector3Add(Vector3Add(p.position, rx), uy);
            Vector3 c3 = Vector3Add(Vector3Subtract(p.position, rx), uy);

            rlColor4ub(col.r, col.g, col.b, col.a);
            rlVertex3f(c0.x, c0.y, c0.z);
            rlVertex3f(c1.x, c1.y, c1.z);
            rlVertex3f(c2.x, c2.y, c2.z);
            rlVertex3f(c3.x, c3.y, c3.z);
        }
        rlEnd();

        rlEnableDepthMask();
        EndBlendMode();
    }

private:
    float radius_ = 16.0f;
    float floorY_ = 0.0f;
    float ceilY_  = 5.0f;
    float maxConc_ = 1.0f;
    float regenRate_ = 0.18f; // how fast grazed sources recover

    std::vector<Blob>     blobs_;
    std::vector<Particle> particles_;

    static float randf() { return (float)GetRandomValue(0, 10000) / 10000.0f; }

    Vector3 randomPointInDish(float margin) const
    {
        float angle = randf() * 2.0f * PI;
        float rXZ   = std::max(0.0f, radius_ - margin) * std::sqrt(randf());
        float y     = floorY_ + randf() * (ceilY_ - floorY_);
        return { rXZ * std::cos(angle), y, rXZ * std::sin(angle) };
    }

    Vector3 sampleByConcentration() const
    {
        Vector3 best = randomPointInDish(0.6f);
        float bestC = concentrationAt(best);
        for (int attempt = 0; attempt < 12; attempt++)
        {
            Vector3 cand = randomPointInDish(0.6f);
            float c = concentrationAt(cand);
            if (randf() < c / maxConc_)
                return cand;
            if (c > bestC) { bestC = c; best = cand; }
        }
        return best;
    }

    void respawn(Particle &p) const
    {
        p.position = sampleByConcentration();
        p.velocity = { (randf() - 0.5f) * 0.4f,
                       (randf() - 0.5f) * 0.4f,
                       (randf() - 0.5f) * 0.4f };
        p.conc     = concentrationAt(p.position) / maxConc_;
        p.size     = 0.03f + randf() * 0.05f;
        p.age      = 0.0f;
        p.lifespan = 2.5f + randf() * 4.0f;
    }

    void reflectIntoDish(Vector3 &pos, Vector3 &vel) const
    {
        if (pos.y < floorY_) { pos.y = floorY_; vel.y = std::fabs(vel.y); }
        if (pos.y > ceilY_)  { pos.y = ceilY_;  vel.y = -std::fabs(vel.y); }

        float r = std::sqrt(pos.x * pos.x + pos.z * pos.z);
        if (r > radius_ && r > 1e-4f)
        {
            float scale = radius_ / r;
            pos.x *= scale;
            pos.z *= scale;
            float nx = pos.x / radius_, nz = pos.z / radius_;
            float vn = vel.x * nx + vel.z * nz;
            if (vn > 0.0f) { vel.x -= 2.0f * vn * nx; vel.z -= 2.0f * vn * nz; }
        }
    }

    static float lifeFade(const Particle &p)
    {
        float t = (p.lifespan > 1e-4f) ? p.age / p.lifespan : 1.0f;
        float fadeIn  = std::min(1.0f, t / 0.15f);
        float fadeOut = std::min(1.0f, (1.0f - t) / 0.25f);
        return std::max(0.0f, std::min(fadeIn, fadeOut));
    }

    static Color concentrationColor(float conc, float alpha)
    {
        float t = std::min(1.0f, std::max(0.0f, conc));
        unsigned char r = (unsigned char)(60.0f  + t * 195.0f);
        unsigned char g = (unsigned char)(170.0f + t * 85.0f);
        unsigned char b = (unsigned char)(150.0f - t * 110.0f);
        unsigned char a = (unsigned char)(std::min(1.0f, std::max(0.0f, alpha)) * 255.0f);
        return { r, g, b, a };
    }
};

#endif
