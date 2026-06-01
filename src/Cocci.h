#ifndef MICRO3D_COCCI_H
#define MICRO3D_COCCI_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <raylib.h>
#include <raymath.h>

#include "PhysicsBody.h"

// A staphylococcus-like cluster of spherical cells. 
class CocciCluster : public PhysicsBody
{
public:
    struct CellMeta
    {
        int centerIndex;
        int startIndex;
        int endIndex;
        float radius; // Track radius per cell for rendering accuracy
    };

    CocciCluster(Vector3 startPos,
                 int   numCells   = 6,
                 float cellRadius = 0.16f,
                 float stiffness  = 75.0f,
                 float damping    = 3.5f)
        : time(0.0f)
    {
        const int numMembraneNodes = 12;
        nodes.reserve((size_t)numCells * (1 + numMembraneNodes));

        for (int c = 0; c < numCells; c++)
        {
            int cellStartIdx = (int)nodes.size();

            Vector3 jitter = {
                (float)GetRandomValue(-10, 10) * 0.015f,
                (float)GetRandomValue(-10, 10) * 0.015f,
                (float)GetRandomValue(-10, 10) * 0.015f
            };
            Vector3 center = Vector3Add(startPos, jitter);

            int centerIdx = cellStartIdx;
            nodes.push_back(Node(center, 0.2f));

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
                nodes.push_back(Node(pos, 0.05f));
            }
            int endIdx = (int)nodes.size() - 1;

            cells.push_back({centerIdx, startIdx, endIdx, cellRadius});
        }

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
                        addSpring(i, j, stiffness * 0.7f, damping);
                }
        }

        float adhesionK = 20.0f; 
        for (size_t i = 0; i < cells.size(); i++)
            for (size_t j = i + 1; j < cells.size(); j++)
            {
                addSpring(cells[i].centerIndex, cells[j].centerIndex, adhesionK, damping * 1.2f);
                springs.back().rest_length = cellRadius * 1.75f;
            }
    }

    std::vector<CellMeta> &getCells() { return cells; }

    void actuate(float dt)
    {
        time += dt;

        const Vector3 fluidCurrent = {0.08f, 0.0f, -0.04f};

        for (auto &n : nodes)
        {
            n.velocity = Vector3Scale(n.velocity, 0.80f);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(fluidCurrent, dt));
        }

        for (auto &cell : cells)
        {
            Vector3 kick = {
                (float)GetRandomValue(-100, 100) / 100.0f,
                (float)GetRandomValue(-100, 100) / 100.0f,
                (float)GetRandomValue(-100, 100) / 100.0f
            };

            kick = Vector3Scale(kick, 2.5f * dt);
            nodes[cell.centerIndex].velocity = Vector3Add(nodes[cell.centerIndex].velocity, kick);
        }
    }

    Vector3 getCenterPosition() const
    {
        if (cells.empty()) return Vector3Zero();

        Vector3 avg = Vector3Zero();
        for (const auto &c : cells)
            avg = Vector3Add(avg, nodes[c.centerIndex].position);
        return Vector3Scale(avg, 1.0f / (float)cells.size());
    }

    void respawn(Vector3 hunterPos, float distance, float restY = 0.0f)
    {
        float angle = (float)GetRandomValue(0, 360) * DEG2RAD;
        Vector3 newClusterCenter = {
            hunterPos.x + std::cos(angle) * distance,
            restY,
            hunterPos.z + std::sin(angle) * distance
        };

        for (size_t c = 0; c < cells.size(); c++)
        {
            auto &cell = cells[c];

            Vector3 cellJitter = {
                (float)GetRandomValue(-10, 10) * 0.015f,
                (float)GetRandomValue(-10, 10) * 0.015f,
                (float)GetRandomValue(-10, 10) * 0.015f
            };
            Vector3 newCellCenter = Vector3Add(newClusterCenter, cellJitter);

            nodes[cell.centerIndex].position = newCellCenter;
            nodes[cell.centerIndex].velocity = Vector3Zero();
            nodes[cell.centerIndex].force = Vector3Zero();
            nodes[cell.centerIndex].acceleration = Vector3Zero();

            int numMembraneNodes = cell.endIndex - cell.startIndex + 1;
            for (int i = 0; i < numMembraneNodes; i++)
            {
                float t     = (float)i / (float)(numMembraneNodes - 1);
                float y     = 1.0f - (t * 2.0f);
                float ringR = std::sqrt(std::max(0.0f, 1.0f - y * y));
                float theta = PI * (1.0f + std::sqrt(5.0f)) * (float)i;

                float x = std::cos(theta) * ringR;
                float z = std::sin(theta) * ringR;

                Vector3 perfectOffset = Vector3Scale({x, y, z}, cell.radius);
                Vector3 newMembranePos = Vector3Add(newCellCenter, perfectOffset);

                int nodeIdx = cell.startIndex + i;
                nodes[nodeIdx].position = newMembranePos;
                nodes[nodeIdx].velocity = Vector3Zero();
                nodes[nodeIdx].force = Vector3Zero();
                nodes[nodeIdx].acceleration = Vector3Zero();
            }
        }
    }

    void draw()
    {

        for (const auto &cell : cells)
        {
            Vector3 cellPos = nodes[cell.centerIndex].position;
            
            Color coreColor = { 40, 140, 220, 160 };
            Color auraColor = { 100, 210, 255, 60 }; 

            DrawSphere(cellPos, cell.radius, coreColor);
            DrawSphere(cellPos, cell.radius * 1.15f, auraColor); // Slight outer glow effect
        }

    }

private:
    std::vector<CellMeta> cells;
    float time;
};

#endif // MICRO3D_COCCI_H