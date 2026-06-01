#include "BoidBehavior.h"

#include <cmath>


static Vector3 clampLength(Vector3 v, float maxLen)
{
    float len = Vector3Length(v);
    if (len > maxLen && len > 1e-6f)
        return Vector3Scale(Vector3Normalize(v), maxLen);
    return v;
}


BoidBehavior::BoidBehavior(float sepR,float aliR,float cohR,float sepW,float aliW,float cohW,float maxF)
    :separationRadius(sepR), alignmentRadius(aliR), cohesionRadius(cohR), separationWeight(sepW), alignmentWeight(aliW), cohesionWeight(cohW), maxForce(maxF) {}

Vector3 BoidBehavior::steer(int selfIndex, const std::vector<BoidState> &flock) const
{
    const BoidState &self = flock[selfIndex];

    Vector3 sep = Vector3Zero();
    Vector3 ali = Vector3Zero();
    Vector3 coh = Vector3Zero();
    int sepCount = 0, aliCount = 0, cohCount = 0;

    for (int i = 0; i < (int)flock.size(); i++)
    {
        if (i == selfIndex) {
            continue;
        }

        const BoidState &other = flock[i];
        Vector3 diff = Vector3Subtract(self.position, other.position);
        float dist   = Vector3Length(diff);

        if (dist < separationRadius && dist > 1e-6f) { //separation == weighted by inverse distance
            sep = Vector3Add(sep, Vector3Scale(Vector3Normalize(diff), 1.0f / dist));
            sepCount++;
        }
        if (dist < alignmentRadius) { //align == match neighbors' velocity
            ali = Vector3Add(ali, other.velocity);
            aliCount++;
        }
        if (dist < cohesionRadius) { //cohesion == move toward neighbors' centroid
            coh = Vector3Add(coh, other.position);
            cohCount++;
        }
    }

    Vector3 steering = Vector3Zero();

    if (sepCount > 0) {
        sep = Vector3Scale(sep, 1.0f / sepCount);
        sep = clampLength(sep, maxForce);
        steering = Vector3Add(steering, Vector3Scale(sep, separationWeight));
    }
    if (aliCount > 0) {
        ali = Vector3Scale(ali, 1.0f / aliCount);
        ali = Vector3Subtract(ali, self.velocity); // velocity error
        ali = clampLength(ali, maxForce);
        steering = Vector3Add(steering, Vector3Scale(ali, alignmentWeight));
    }
    if (cohCount > 0) {
        coh = Vector3Scale(coh, 1.0f / cohCount);
        coh = Vector3Subtract(coh, self.position); // toward neighbor centroid
        coh = clampLength(coh, maxForce);
        steering = Vector3Add(steering, Vector3Scale(coh, cohesionWeight));
    }

    if (predatorAvoidanceEnabled) {
        // Old behavior: avoid one predatorPosition.
        // Vector3 away = Vector3Subtract(self.position, predatorPosition);
        int predatorCount = predatorPositions.empty() ? 1 : (int)predatorPositions.size();
        for (int p = 0; p < predatorCount; p++) {
            Vector3 predator = predatorPositions.empty() ? predatorPosition : predatorPositions[p];
            Vector3 away = Vector3Subtract(self.position, predator);
            away.y = 0.0f;
            float dist = Vector3Length(away);
            if (dist < predatorAvoidRadius && dist > 1e-6f) {
                float urgency = 1.0f - dist / predatorAvoidRadius;
                Vector3 predatorAvoid = Vector3Scale(
                    Vector3Normalize(away),
                    urgency * urgency);
                predatorAvoid = clampLength(predatorAvoid, maxForce);
                steering = Vector3Add(steering, Vector3Scale(predatorAvoid, predatorAvoidWeight));
            }
        }
    }

    return steering;
}


BoidForceGenerator::BoidForceGenerator(const std::vector<BoidState> *flockPtr, int index, int bodyNodes, const BoidBehavior *beh)
    :flock(flockPtr), selfIndex(index), bodyNodeCount(bodyNodes), behavior(beh) {}

void BoidForceGenerator::apply(PhysicsBody &body, float /*dt*/)
{
    if (!flock || !behavior || (int)flock->size() <= selfIndex) {
        return;
    }

    Vector3 force = behavior->steer(selfIndex, *flock);

    //suppress vertical steering; let buoyancy handle y (for now)
    force.y *= 0.1f;

    auto &nodes = body.getNodes();
    for (int i = 0; i < bodyNodeCount && i < (int)nodes.size(); i++) {
        nodes[i].force = Vector3Add(nodes[i].force, force);
    }
}
