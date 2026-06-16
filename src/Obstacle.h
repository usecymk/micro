#ifndef MICRO3D_OBSTACLE_H
#define MICRO3D_OBSTACLE_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>

#include "Node.h"

enum class ObstacleShape
{
    SPHERE,
    BOX
};

class Obstacle
{
public:
    ObstacleShape shape;
    Vector3       center;
    float         radius       = 0.0f;
    Vector3       halfExtents  = {0.0f, 0.0f, 0.0f};
    Color         fillColor    = {42, 58, 72, 215};
    Color         wireColor    = {90, 130, 155, 180};

    static Obstacle makeSphere(Vector3 pos, float r)
    {
        Obstacle o;
        o.shape  = ObstacleShape::SPHERE;
        o.center = pos;
        o.radius = r;
        return o;
    }

    static Obstacle makeBox(Vector3 pos, Vector3 halfSize)
    {
        Obstacle o;
        o.shape        = ObstacleShape::BOX;
        o.center       = pos;
        o.halfExtents  = halfSize;
        return o;
    }

    Vector3 getCenter() const { return center; }

    float outerRadius() const
    {
        if (shape == ObstacleShape::SPHERE)
            return radius;
        return Vector3Length(halfExtents);
    }

    // Signed distance to the obstacle surface (negative when inside).
    float signedDistanceToSurface(Vector3 point) const
    {
        if (shape == ObstacleShape::SPHERE)
        {
            return Vector3Distance(point, center) - radius;
        }

        Vector3 rel = Vector3Subtract(point, center);
        Vector3 q = {
            std::abs(rel.x) - halfExtents.x,
            std::abs(rel.y) - halfExtents.y,
            std::abs(rel.z) - halfExtents.z
        };
        Vector3 qMax = {
            std::max(q.x, 0.0f),
            std::max(q.y, 0.0f),
            std::max(q.z, 0.0f)
        };
        float outside = Vector3Length(qMax);
        float inside = std::min(std::max(std::max(q.x, q.y), q.z), 0.0f);
        return outside + inside;
    }

    Vector3 closestPointOnSurface(Vector3 point) const
    {
        if (shape == ObstacleShape::SPHERE)
        {
            Vector3 delta = Vector3Subtract(point, center);
            float dist = Vector3Length(delta);
            if (dist < 1e-6f)
                return Vector3Add(center, {radius, 0.0f, 0.0f});
            return Vector3Add(center, Vector3Scale(delta, radius / dist));
        }

        Vector3 min = Vector3Subtract(center, halfExtents);
        Vector3 max = Vector3Add(center, halfExtents);
        return {
            Clamp(point.x, min.x, max.x),
            Clamp(point.y, min.y, max.y),
            Clamp(point.z, min.z, max.z)
        };
    }

    void resolveNode(Node &n, float bodyRadius, float restitution = 0.35f) const
    {
        if (shape == ObstacleShape::SPHERE)
            resolveSphereNode(n, bodyRadius, restitution);
        else
            resolveBoxNode(n, bodyRadius, restitution);
    }

    void draw() const
    {
        if (shape == ObstacleShape::SPHERE)
        {
            DrawSphere(center, radius, fillColor);
            DrawSphereWires(center, radius, 14, 10, wireColor);
            return;
        }

        DrawCube(center, halfExtents.x * 2.0f, halfExtents.y * 2.0f, halfExtents.z * 2.0f, fillColor);
        DrawCubeWires(center, halfExtents.x * 2.0f, halfExtents.y * 2.0f, halfExtents.z * 2.0f, wireColor);
    }

private:
    void resolveSphereNode(Node &n, float bodyRadius, float restitution) const
    {
        Vector3 delta = Vector3Subtract(n.position, center);
        float dist = Vector3Length(delta);
        float combined = radius + bodyRadius;

        if (dist >= combined)
            return;

        Vector3 normal;
        if (dist > 1e-6f)
            normal = Vector3Scale(delta, 1.0f / dist);
        else
            normal = {1.0f, 0.0f, 0.0f};

        float penetration = combined - std::max(dist, 0.0f);
        n.position = Vector3Add(n.position, Vector3Scale(normal, penetration));

        float vn = Vector3DotProduct(n.velocity, normal);
        if (vn < 0.0f)
            n.velocity = Vector3Subtract(n.velocity, Vector3Scale(normal, (1.0f + restitution) * vn));
    }

    void resolveBoxNode(Node &n, float bodyRadius, float restitution) const
    {
        Vector3 closest = closestPointOnSurface(n.position);
        Vector3 delta = Vector3Subtract(n.position, closest);
        float dist = Vector3Length(delta);

        if (dist >= bodyRadius)
            return;

        Vector3 normal;
        if (dist > 1e-6f)
        {
            normal = Vector3Scale(delta, 1.0f / dist);
        }
        else
        {
            Vector3 rel = Vector3Subtract(n.position, center);
            float px = halfExtents.x - std::abs(rel.x);
            float py = halfExtents.y - std::abs(rel.y);
            float pz = halfExtents.z - std::abs(rel.z);

            if (px <= py && px <= pz)
                normal = {rel.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f};
            else if (py <= pz)
                normal = {0.0f, rel.y >= 0.0f ? 1.0f : -1.0f, 0.0f};
            else
                normal = {0.0f, 0.0f, rel.z >= 0.0f ? 1.0f : -1.0f};
        }

        float penetration = bodyRadius - dist;
        n.position = Vector3Add(n.position, Vector3Scale(normal, penetration));

        float vn = Vector3DotProduct(n.velocity, normal);
        if (vn < 0.0f)
            n.velocity = Vector3Subtract(n.velocity, Vector3Scale(normal, (1.0f + restitution) * vn));
    }
};

inline void resolveObstacleCollisions(const std::vector<Obstacle> &obstacles,
                                      std::vector<Node> &nodes,
                                      float bodyRadius,
                                      float restitution = 0.35f)
{
    for (auto &n : nodes)
        for (const auto &obs : obstacles)
            obs.resolveNode(n, bodyRadius, restitution);
}

inline void resolveObstacleCollisions(const std::vector<Obstacle> &obstacles,
                                      Node &node,
                                      float bodyRadius,
                                      float restitution = 0.35f)
{
    for (const auto &obs : obstacles)
        obs.resolveNode(node, bodyRadius, restitution);
}

#endif
