#ifndef MICRO3D_NODE_H
#define MICRO3D_NODE_H

#include <raylib.h>
#include <raymath.h>

// A point mass in a deformable body.
//
// `volume` is the fluid volume this node displaces; it is used by
// BuoyancyForce (and any future fluid force) to compute hydrostatic
// effects without having to know the body's topology.
//
// By default we choose volume so that a unit-mass node is neutrally
// buoyant in fresh water (rho = 1000 kg/m^3): V = m / rho = 1e-3 m^3.
struct Node
{
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    Vector3 acceleration;
    float mass;
    float volume;

    Node(Vector3 pos, float m = 1.0f, float v = 1.0e-3f)
        : position(pos),
          velocity(Vector3Zero()),
          force(Vector3Zero()),
          acceleration(Vector3Zero()),
          mass(m),
          volume(v) {}
};

// A damped Hookean spring between two nodes.
struct Spring
{
    Node *nodeA;
    Node *nodeB;
    float rest_length;
    float stiffness;
    float damping;

    Spring(Node *A, Node *B, float rl = 1.0f, float s = 1.0f, float d = 1.0f)
        : nodeA(A), nodeB(B), rest_length(rl), stiffness(s), damping(d) {}
};

#endif // MICRO3D_NODE_H
