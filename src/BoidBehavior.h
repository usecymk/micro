#ifndef MICRO3D_BOID_BEHAVIOR_H
#define MICRO3D_BOID_BEHAVIOR_H

#include <vector>
#include <raylib.h>
#include <raymath.h>

#include "ForceGenerator.h"
#include "PhysicsBody.h"

//centroid & av velocity of boid's body nodes 
    //capture @ each frame.
struct BoidState
{
    Vector3 position;
    Vector3 velocity;
};

//three-rule boid steering (separation, alignment, cohesion).
class BoidBehavior
{
public:
    float separationRadius = 1.8f;
    float alignmentRadius = 3.5f;
    float cohesionRadius = 3.5f;
    float separationWeight = 3.0f;
    float alignmentWeight = 1.0f;
    float cohesionWeight = 0.8f;
    float maxForce = 2.5f;

    BoidBehavior() = default;
    BoidBehavior(float sepR, float aliR, float cohR, float sepW, float aliW, float cohW, float maxF = 2.5f);

    //ret combined steering force for flock[selfIndex]
    Vector3 steer(int selfIndex, const std::vector<BoidState> &flock) const;
};

//ForceGenerator that applies boid steering to a body's front (body) nodes.
    //shared flock-state vector is owned externally and updated once per frame
    //b4 physics (all bodies see consistent snapshot]
class BoidForceGenerator : public ForceGenerator
{
    const std::vector<BoidState> *flock;
    int selfIndex;
    int bodyNodeCount;
    const BoidBehavior *behavior;

public:
    BoidForceGenerator(const std::vector<BoidState> *flockPtr, int index, int bodyNodes, const BoidBehavior *beh);
    void apply(PhysicsBody &body, float dt) override;
};

#endif // MICRO3D_BOID_BEHAVIOR_H
