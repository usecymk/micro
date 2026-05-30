#ifndef MICRO3D_COCCI_H
#define MICRO3D_COCCI_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"

// A staphylococcus-like cluster of spherical cells. Each cell is a nucleus
// node ringed by membrane nodes (fibonacci sphere), and cells are loosely
// adhered to one another by long, soft springs between their nuclei.
//
// As with `Amoeba`, motility lives in `actuate()` (velocity damping,
// fluid current, brownian kicks). Gravity is omitted by design - the
// cluster lives suspended in the medium - so don't attach a `GravityForce`.
class CocciCluster : public PhysicsBody
{
public:
    struct CellMeta
    {
        int centerIndex;
        int startIndex;
        int endIndex;
    };

    CocciCluster(Vector3 startPos,
                 int   numCells   = 6,
                 float cellRadius = 0.5f,
                 float stiffness  = 250.0f,
                 float damping    = 30.0f)
        : time(0.0f)
    {
        const int numMembraneNodes = 16;
        nodes.reserve((size_t)numCells * (1 + numMembraneNodes));

        for (int c = 0; c < numCells; c++)
        {
            int cellStartIdx = (int)nodes.size();

            Vector3 jitter = {
                (float)GetRandomValue(-10, 10) * 0.05f,
                (float)GetRandomValue(-10, 10) * 0.05f,
                (float)GetRandomValue(-10, 10) * 0.05f
            };
            Vector3 center = Vector3Add(startPos, jitter);

            int centerIdx = cellStartIdx;
            nodes.push_back(Node(center, 0.4f));

            int startIdx = (int)nodes.size();
            for (int i = 0; i < numMembraneNodes; i++)
            {
                float t     = (float)i / (float)(numMembraneNodes - 1);
                float y     = 1.0f - (t * 2.0f);
                float ringR = std::sqrt(std::max(0.0f, 1.0f - y * y));
                float theta = PI * (1.0f + std::sqrt(5.0f)) * (float)i;

                float x = std::cos(theta) * ringR;
                float z = std::sin(theta) * ringR;

                Vector3 pos = Vector3Add(center, Vector3Scale({x, y, z}, cellRadius));
                nodes.push_back(Node(pos, 0.15f));
            }
            int endIdx = (int)nodes.size() - 1;

            cells.push_back({centerIdx, startIdx, endIdx});
        }

        // Intra-cell springs: nucleus -> membrane, and short surface bonds.
        for (const auto &cell : cells)
        {
            for (int i = cell.startIndex; i <= cell.endIndex; i++)
                addSpring(cell.centerIndex, i, stiffness, damping);

            float expected  = std::sqrt(4.0f * PI * cellRadius * cellRadius / (float)numMembraneNodes);
            float threshold = expected * 1.6f;
            for (int i = cell.startIndex; i <= cell.endIndex; i++)
                for (int j = i + 1; j <= cell.endIndex; j++)
                {
                    float d = Vector3Distance(nodes[i].position, nodes[j].position);
                    if (d < threshold)
                        addSpring(i, j, stiffness * 0.8f, damping);
                }
        }

        // Inter-cell adhesion springs hold the cluster together loosely.
        float adhesionK = 45.0f;
        for (size_t i = 0; i < cells.size(); i++)
            for (size_t j = i + 1; j < cells.size(); j++)
            {
                addSpring(cells[i].centerIndex, cells[j].centerIndex, adhesionK, damping * 1.5f);
                springs.back().rest_length = cellRadius * 1.9f;
            }
    }

    std::vector<CellMeta> &getCells() { return cells; }

    void actuate(float dt)
    {
        time += dt;

        const Vector3 fluidCurrent = {0.3f, 0.0f, -0.2f};

        // viscous damping + ambient current
        for (auto &n : nodes)
        {
            n.velocity = Vector3Scale(n.velocity, 0.82f);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(fluidCurrent, dt));
        }

        // brownian jitter on each cell's nucleus
        for (auto &cell : cells)
        {
            Vector3 kick = {
                (float)GetRandomValue(-100, 100) / 250.0f,
                (float)GetRandomValue(-100, 100) / 250.0f,
                (float)GetRandomValue(-100, 100) / 250.0f
            };
            kick = Vector3Scale(kick, 55.0f * dt);
            nodes[cell.centerIndex].velocity =
                Vector3Add(nodes[cell.centerIndex].velocity, kick);
        }
    }

    Vector3 getCenterPosition() const
    {
        if (cells.empty())
            return Vector3Zero();

        Vector3 avg = Vector3Zero();
        for (const auto &c : cells)
            avg = Vector3Add(avg, nodes[c.centerIndex].position);
        return Vector3Scale(avg, 1.0f / (float)cells.size());
    }

    // Teleport the entire cluster to a fresh location around `hunterPos`,
    // killing momentum so the springs don't snap back violently.
    void respawn(Vector3 hunterPos, float distance, float restY = 0.0f)
    {
        float angle = (float)GetRandomValue(0, 360) * DEG2RAD;
        Vector3 newCenter = {
            hunterPos.x + std::cos(angle) * distance,
            restY,
            hunterPos.z + std::sin(angle) * distance
        };

        Vector3 shift = Vector3Subtract(newCenter, getCenterPosition());
        for (auto &n : nodes)
        {
            n.position = Vector3Add(n.position, shift);
            n.velocity = Vector3Zero();
            n.force = Vector3Zero();
            n.acceleration = Vector3Zero();
        }
    }

    void draw()
    {
        for (const auto &cell : cells)
        {
            for (int i = cell.startIndex; i <= cell.endIndex; i++)
                DrawSphere(nodes[i].position, 0.08f, SKYBLUE);
            DrawSphere(nodes[cell.centerIndex].position, 0.15f, BLUE);
        }
        for (auto &s : springs)
            DrawLine3D(s.nodeA->position, s.nodeB->position, Fade(SKYBLUE, 0.2f));
    }

private:
    std::vector<CellMeta> cells;
    float time;
};

#endif
