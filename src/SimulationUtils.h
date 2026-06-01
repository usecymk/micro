#ifndef MICRO3D_SIMULATION_UTILS_H
#define MICRO3D_SIMULATION_UTILS_H

#include <memory>
#include <vector>
#include <cmath>

#include <raylib.h>
#include <raymath.h>

#include "Amoeba.h"
#include "Bacteria.h"
#include "Cocci.h"
#include "Node.h"
#include "PetriDish.h"

static Vector3 randomBacteriaSpawn(float radius, float y, Vector3 avoidPos)
{
    Vector3 spawn = {0.0f, y, 0.0f};
    for (int attempt = 0; attempt < 20; attempt++)
    {
        float angle = (float)GetRandomValue(0, 359) * DEG2RAD;
        float unitR = std::sqrt((float)GetRandomValue(0, 1000) / 1000.0f);
        float r = radius * 0.82f * unitR;
        spawn = {cosf(angle) * r, y, sinf(angle) * r};

        Vector3 away = Vector3Subtract(spawn, avoidPos);
        away.y = 0.0f;
        if (Vector3Length(away) > 2.5f)
            break;
    }

    return spawn;
}

static void translateBacteria(Bacteria &b, Vector3 delta)
{
    for (auto &n : b.getNodes())
        n.position = Vector3Add(n.position, delta);
}

static void translateNodes(std::vector<Node> &nodes, Vector3 delta)
{
    for (auto &n : nodes)
        n.position = Vector3Add(n.position, delta);
}

static void dampCollisionVelocity(Bacteria &b, Vector3 normal, bool positiveNormal)
{
    for (auto &n : b.getNodes())
    {
        float vn = Vector3DotProduct(n.velocity, normal);
        if ((positiveNormal && vn > 0.0f) || (!positiveNormal && vn < 0.0f))
            n.velocity = Vector3Subtract(n.velocity, Vector3Scale(normal, vn * 0.55f));
    }
}

static void dampNodeCollisionVelocity(std::vector<Node> &nodes, Vector3 normal, bool positiveNormal)
{
    for (auto &n : nodes)
    {
        float vn = Vector3DotProduct(n.velocity, normal);
        if ((positiveNormal && vn > 0.0f) || (!positiveNormal && vn < 0.0f))
            n.velocity = Vector3Subtract(n.velocity, Vector3Scale(normal, vn * 0.55f));
    }
}

static void resolveBacteriaSoftCollisions(std::vector<Bacteria> &flock,
                                          const PetriDish &dish)
{
    const float desiredSpacing = 0.22f;
    const float correctionPercent = 0.45f;

    for (int i = 0; i < (int)flock.size(); i++)
    {
        for (int j = i + 1; j < (int)flock.size(); j++)
        {
            Vector3 a = flock[i].getCenterOfMass();
            Vector3 b = flock[j].getCenterOfMass();
            Vector3 delta = Vector3Subtract(b, a);
            float dist = Vector3Length(delta);

            if (dist <= 1e-5f || dist >= desiredSpacing)
                continue;

            Vector3 normal = Vector3Scale(delta, 1.0f / dist);
            float overlap = desiredSpacing - dist;
            Vector3 correction = Vector3Scale(normal, overlap * correctionPercent * 0.5f);

            translateBacteria(flock[i], Vector3Negate(correction));
            translateBacteria(flock[j], correction);
            dampCollisionVelocity(flock[i], normal, true);
            dampCollisionVelocity(flock[j], normal, false);

            dish.applyBoundary(flock[i].getNodes());
            dish.applyBoundary(flock[j].getNodes());
        }
    }
}

static void resolveBodySoftCollision(std::vector<Node> &aNodes,
                                     Vector3 aCenter,
                                     float aRadius,
                                     std::vector<Node> &bNodes,
                                     Vector3 bCenter,
                                     float bRadius,
                                     const PetriDish &dish,
                                     float correctionPercent = 0.45f)
{
    Vector3 delta = Vector3Subtract(bCenter, aCenter);
    float dist = Vector3Length(delta);
    float minDist = aRadius + bRadius;

    if (dist <= 1e-5f || dist >= minDist)
        return;

    Vector3 normal = Vector3Scale(delta, 1.0f / dist);
    float overlap = minDist - dist;
    Vector3 correction = Vector3Scale(normal, overlap * correctionPercent * 0.5f);

    translateNodes(aNodes, Vector3Negate(correction));
    translateNodes(bNodes, correction);
    dampNodeCollisionVelocity(aNodes, normal, true);
    dampNodeCollisionVelocity(bNodes, normal, false);
    dish.applyBoundary(aNodes);
    dish.applyBoundary(bNodes);
}

static void resolveAmoebaBacteriaSoftCollisions(std::vector<std::unique_ptr<Amoeba>> &amoebas,
                                                std::vector<Bacteria> &flock,
                                                const PetriDish &dish)
{
    const float amoebaRadius = 0.58f;
    const float bacteriaRadius = 0.08f;
    const float captureDistance = 1.05f;

    for (auto &amoeba : amoebas)
    {
        for (auto &bacteria : flock)
        {
            float dist3D = Vector3Distance(amoeba->getCenterPosition(), bacteria.getCenterOfMass());
            if (dist3D <= captureDistance)
                continue;

            resolveBodySoftCollision(
                amoeba->getNodes(),
                amoeba->getCenterPosition(),
                amoebaRadius,
                bacteria.getNodes(),
                bacteria.getCenterOfMass(),
                bacteriaRadius,
                dish,
                0.35f);
        }
    }
}

static void resolveAmoebaSoftCollisions(std::vector<std::unique_ptr<Amoeba>> &amoebas,
                                        const PetriDish &dish)
{
    const float amoebaRadius = 0.72f;

    for (int i = 0; i < (int)amoebas.size(); i++)
    {
        for (int j = i + 1; j < (int)amoebas.size(); j++)
        {
            resolveBodySoftCollision(
                amoebas[i]->getNodes(),
                amoebas[i]->getCenterPosition(),
                amoebaRadius,
                amoebas[j]->getNodes(),
                amoebas[j]->getCenterPosition(),
                amoebaRadius,
                dish,
                0.40f);
        }
    }
}

static void consumeCapturedBacteria(std::vector<std::unique_ptr<Amoeba>> &amoebas,
                                    std::vector<Bacteria> &flock,
                                    CocciCluster &cocci,
                                    const PetriDish &dish)
{
    const float captureDistance = 1.05f;

    for (auto &amoeba : amoebas)
    {
        if (Vector3Distance(amoeba->getCenterPosition(), cocci.getCenterPosition()) <= captureDistance + 0.5f)
        {
            cocci.respawn(amoeba->getCenterPosition(), dish.radius * 0.85f, dish.floorY + 2.0f);
            amoeba->onConsumedPrey();
            continue;
        }

        int capturedIndex = -1;
        float closestCaptureDist = captureDistance;

        for (int i = 0; i < (int)flock.size(); i++)
        {
            float dist3D = Vector3Distance(amoeba->getCenterPosition(), flock[i].getCenterOfMass());
            if (dist3D <= closestCaptureDist)
            {
                closestCaptureDist = dist3D;
                capturedIndex = i;
            }
        }

        if (capturedIndex >= 0)
        {
            flock[capturedIndex].reset(
                randomBacteriaSpawn(dish.radius, dish.floorY + 2.0f, amoeba->getCenterPosition()));
            amoeba->onConsumedPrey();
        }
    }
}

static Vector3 closestPredatorPosition(Vector3 pos, const std::vector<Vector3> &predators)
{
    if (predators.empty())
        return Vector3Zero();

    Vector3 closest = predators[0];
    float closestDist = Vector3Distance(pos, closest);
    for (int i = 1; i < (int)predators.size(); i++)
    {
        float dist = Vector3Distance(pos, predators[i]);
        if (dist < closestDist)
        {
            closestDist = dist;
            closest = predators[i];
        }
    }

    return closest;
}

#endif // MICRO3D_SIMULATION_UTILS_H
