#ifndef MICRO3D_CUBE_H
#define MICRO3D_CUBE_H

#include "PhysicsBody.h"

class Cube : public PhysicsBody
{
public:
    Cube(float size = 1.0f, float stiffness = 50.0f, float damping = 5.0f)
    {
        nodes.reserve(8);
        float h = size / 2.0f;
        nodes.push_back(Node(Vector3{-h, -h, -h}));
        nodes.push_back(Node(Vector3{h, -h, -h}));
        nodes.push_back(Node(Vector3{-h, h, -h}));
        nodes.push_back(Node(Vector3{h, h, -h}));
        nodes.push_back(Node(Vector3{-h, -h, h}));
        nodes.push_back(Node(Vector3{h, -h, h}));
        nodes.push_back(Node(Vector3{-h, h, h}));
        nodes.push_back(Node(Vector3{h, h, h}));

        // 12 edges
        addSpring(0, 1, stiffness, damping);
        addSpring(2, 3, stiffness, damping);
        addSpring(4, 5, stiffness, damping);
        addSpring(6, 7, stiffness, damping);
        addSpring(0, 2, stiffness, damping);
        addSpring(1, 3, stiffness, damping);
        addSpring(4, 6, stiffness, damping);
        addSpring(5, 7, stiffness, damping);
        addSpring(0, 4, stiffness, damping);
        addSpring(1, 5, stiffness, damping);
        addSpring(2, 6, stiffness, damping);
        addSpring(3, 7, stiffness, damping);

        // 12 face diagonals (2 per face)
        addSpring(0, 3, stiffness, damping);
        addSpring(1, 2, stiffness, damping);
        addSpring(4, 7, stiffness, damping);
        addSpring(5, 6, stiffness, damping);
        addSpring(0, 5, stiffness, damping);
        addSpring(1, 4, stiffness, damping);
        addSpring(2, 7, stiffness, damping);
        addSpring(3, 6, stiffness, damping);
        addSpring(0, 6, stiffness, damping);
        addSpring(2, 4, stiffness, damping);
        addSpring(1, 7, stiffness, damping);
        addSpring(3, 5, stiffness, damping);

        // 4 body diagonals
        addSpring(0, 7, stiffness, damping);
        addSpring(1, 6, stiffness, damping);
        addSpring(2, 5, stiffness, damping);
        addSpring(3, 4, stiffness, damping);
    }
};

#endif
