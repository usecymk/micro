#pragma once
#include "raylib.h"
#include "raymath.h"
#include <cmath>

constexpr float DISH_RADIUS = 8.0f;
constexpr float DISH_HEIGHT =  5.0f;
constexpr float DISH_FLOOR_Y = 0.0f;   
constexpr float DISH_CEIL_Y = DISH_FLOOR_Y + DISH_HEIGHT;

//colors
static const Color DISH_FLOOR_COLOR  = { 10,  30,  60, 255 };  //navy
static const Color DISH_WALL_COLOR   = { 80, 180, 220, 60  };  //translucent icy-ish blue
static const Color DISH_WIRE_COLOR   = { 120, 210, 255, 180 };  //brighter rim wire
static const Color DISH_LIQUID_COLOR = {  20,  80, 130, 40  };  //light water tint color



inline void DrawPetriDish()
{
    Vector3 base = { 0.0f, DISH_FLOOR_Y, 0.0f };

    //floor circle
    DrawCylinder(base, DISH_RADIUS, DISH_RADIUS, 0.08f, 40, DISH_FLOOR_COLOR);
    DrawCylinderWires(base, DISH_RADIUS, DISH_RADIUS, 0.08f, 40, DISH_WIRE_COLOR);

    //wall cylinder
    float wallThick = 0.18f;
    Vector3 wallBase = base;
    DrawCylinder(wallBase, DISH_RADIUS, DISH_RADIUS, DISH_HEIGHT, 40, DISH_WALL_COLOR);

    //outer rim wire rings
    DrawCylinderWires(wallBase, DISH_RADIUS, DISH_RADIUS, DISH_HEIGHT, 40, DISH_WIRE_COLOR);

    //faint liquid-fill tint (v transparent inner cylinder)
    DrawCylinder(wallBase,
                 DISH_RADIUS - wallThick,
                 DISH_RADIUS - wallThick,
                 DISH_HEIGHT, 40, DISH_LIQUID_COLOR);

    //grid lines on the floor
    int lines = 8;
    float step = (DISH_RADIUS * 2.0f) / lines;
    Color gridCol = { 30, 70, 120, 80 };
    float y = DISH_FLOOR_Y + 0.05f;
    for (int i = 0; i <= lines; i++)
    {
        float t = -DISH_RADIUS + i * step;
        float halfLen = std::sqrt(std::max(0.0f, DISH_RADIUS * DISH_RADIUS - t * t));
        DrawLine3D({ t,  y, -halfLen }, { t,  y,  halfLen }, gridCol);
        DrawLine3D({ -halfLen, y, t  }, {  halfLen, y, t  }, gridCol);
    }
}



//restitution: 0 = fully inelastic, 1 = fully elastic bounce
inline void ApplyDishBoundary(Node& n, float restitution = 0.4f)
{
    //floor
    const float bodyRadius = 0.18f;
    if (n.position.y < DISH_FLOOR_Y + bodyRadius)
    {
        n.position.y = DISH_FLOOR_Y + bodyRadius;
        if (n.velocity.y < 0.0f) {
            n.velocity.y = -n.velocity.y * restitution;
        }
    }

    //ceiling (liquid surface)
    if (n.position.y > DISH_CEIL_Y)
    {
        n.position.y = DISH_CEIL_Y;
        if (n.velocity.y > 0.0f) {
            n.velocity.y = -n.velocity.y * restitution;
        }
    }

    //cylindrical wall 
    float dx = n.position.x;
    float dz = n.position.z;
    float r  = std::sqrt(dx * dx + dz * dz);

    if (r > DISH_RADIUS)
    {
        //push node back to wall surface
        float scale    = DISH_RADIUS / r;
        n.position.x  *= scale;
        n.position.z  *= scale;

        //reflect the rad comp of velocity
        float nx = dx / r;   //outward normal x
        float nz = dz / r;   //outward normal z
        float vn = n.velocity.x * nx + n.velocity.z * nz;  //normal velocity

        if (vn > 0.0f) { //only reflect if moving outward
            n.velocity.x -= (1.0f + restitution) * vn * nx;
            n.velocity.z -= (1.0f + restitution) * vn * nz;
        }
    }
}


//use to put bound on every node
inline void ApplyDishBoundaryAll(std::vector<Node>& nodes, float restitution = 0.4f)
{
    for (auto& n : nodes)
        ApplyDishBoundary(n, restitution);
}