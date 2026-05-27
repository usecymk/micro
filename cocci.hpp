#pragma once
#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>

class CocciCluster : public PhysicsBody
{
public:
    struct CellMeta {
        int centerIndex;
        int startIndex;
        int endIndex;
    };

private:
    std::vector<CellMeta> cells;
    float time;

public:
    CocciCluster(Vector3 startPos, int numCells = 6, float cellRadius = 0.5f, float stiffness = 150.0f, float damping = 3.0f)
    {
        time = 0.0f;
        int numMembraneNodes = 16; 
        
        int totalNodes = numCells * (1 + numMembraneNodes);
        nodes.reserve(totalNodes);

        for (int c = 0; c < numCells; c++)
        {
            int cellStartIdx = nodes.size();

            Vector3 offset = {
                (float)GetRandomValue(-10, 10) * 0.05f,
                (float)GetRandomValue(-10, 10) * 0.05f,
                (float)GetRandomValue(-10, 10) * 0.05f
            };
            Vector3 center = Vector3Add(startPos, offset);
            
            int centerIdx = cellStartIdx;
            nodes.push_back(Node(center, 0.4f)); 

            int startIdx = nodes.size();
            for (int i = 0; i < numMembraneNodes; i++)
            {
                float t = (float)i / (numMembraneNodes - 1);
                float y = 1.0f - (t * 2.0f);
                float radius_at_y = std::sqrt(1.0f - y * y);
                float theta = PI * (1.0f + std::sqrt(5.0f)) * i; 
                
                float x = std::cos(theta) * radius_at_y;
                float z = std::sin(theta) * radius_at_y;

                Vector3 posOffset = Vector3Scale({x, y, z}, cellRadius);
                nodes.push_back(Node(Vector3Add(center, posOffset), 0.15f));
            }
            int endIdx = nodes.size() - 1;

            cells.push_back({centerIdx, startIdx, endIdx});
        }

        for (const auto& cell : cells)
        {
            for (int i = cell.startIndex; i <= cell.endIndex; i++)
            {
                addSpring(cell.centerIndex, i, stiffness, damping);
            }

            float expected_dist = std::sqrt(4.0f * PI * cellRadius * cellRadius / numMembraneNodes);
            float threshold = expected_dist * 1.6f;

            for (int i = cell.startIndex; i <= cell.endIndex; i++)
            {
                for (int j = i + 1; j <= cell.endIndex; j++)
                {
                    float d = Vector3Distance(nodes[i].position, nodes[j].position);
                    if (d < threshold)
                    {
                        addSpring(i, j, stiffness * 0.8f, damping);
                    }
                }
            }
        }

        //--- BIND CELLS INTO A STAPH CLUSTER ---
        float adhesionStiffness = 45.0f; 
        for (size_t i = 0; i < cells.size(); i++)
        {
            for (size_t j = i + 1; j < cells.size(); j++)
            {
                addSpring(cells[i].centerIndex, cells[j].centerIndex, adhesionStiffness, damping * 1.5f);
                springs.back().rest_length = cellRadius * 1.9f; 
            }
        }
    }

    std::vector<CellMeta>& getCells() { return cells; }

    void actuate(float dt)
    {
        time += dt;

        Vector3 buoyancy = {0.0f, 9.81f, 0.0f}; 
        Vector3 fluidCurrent = {0.3f, 0.0f, -0.2f}; 

        for (size_t i = 0; i < nodes.size(); i++)
        {
            Vector3 antiGrav = Vector3Scale(buoyancy, nodes[i].mass);
            nodes[i].force = Vector3Add(nodes[i].force, antiGrav);
            nodes[i].velocity = Vector3Scale(nodes[i].velocity, 0.82f);
            nodes[i].velocity = Vector3Add(nodes[i].velocity, Vector3Scale(fluidCurrent, dt));
        }

        for (auto& cell : cells)
        {
            Vector3 brownianKick = {
                ((float)GetRandomValue(-100, 100) / 175.0f),
                ((float)GetRandomValue(-100, 100) / 175.0f),
                ((float)GetRandomValue(-100, 100) / 175.0f)
            };
            
            brownianKick = Vector3Scale(brownianKick, 55.0f * dt);
            nodes[cell.centerIndex].velocity = Vector3Add(nodes[cell.centerIndex].velocity, brownianKick);
        }
    }
};