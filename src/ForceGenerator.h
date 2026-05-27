#ifndef MICRO3D_FORCE_GENERATOR_H
#define MICRO3D_FORCE_GENERATOR_H

class PhysicsBody;

class ForceGenerator
{
public:
    virtual ~ForceGenerator() = default;
    virtual void apply(PhysicsBody &body, float dt) = 0;
};

#endif
