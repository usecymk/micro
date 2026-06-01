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
        if (i == selfIndex || !flock[i].alive) {
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

    force.y *= 0.8f;

    auto &nodes = body.getNodes();
    for (int i = 0; i < bodyNodeCount && i < (int)nodes.size(); i++) {
        nodes[i].force = Vector3Add(nodes[i].force, force);
    }
}
