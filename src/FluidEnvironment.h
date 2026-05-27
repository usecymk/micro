#ifndef MICRO3D_FLUID_ENVIRONMENT_H
#define MICRO3D_FLUID_ENVIRONMENT_H

#include <raylib.h>

struct FluidEnvironment
{
    float density = 1000.0f;
    float surfaceY = 0.0f;
    Vector3 gravity = {0.0f, -9.81f, 0.0f};
};

#endif
