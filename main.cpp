#include <iostream>
#include <vector>
#include <cmath>
#include <raylib.h>
#include <raymath.h>
#include <Eigen/Sparse>

const Vector3 GLOBAL_GRAVITY = {0.0f, -9.81f, 0.0f};

struct FluidEnvironment
{
    float waterSurfaceY = 5.0f;
    float waterBottomY = -8.0f;
    float fluidDensity = 8.0f;
    float dragNormal = 0.15f;
    float dragTangent = 0.08f;
    float gravityMagnitude = 9.81f;
    Vector3 fluidVelocity = {0.0f, 0.0f, 0.0f};
};

struct Node
{
    Vector3 position, velocity, force, acceleration;
    double mass;
    Node(Vector3 pos, float m = 1.0f) : position(pos), velocity(Vector3Zero()), acceleration(Vector3Zero()), mass(m) {}
};

struct Spring
{
    Node *nodeA, *nodeB;
    float rest_length;
    float stiffness;
    float damping;
    Spring(Node *A, Node *B, float rl = 1.0f, float s = 1.0f, float d = 1.0f) : nodeA(A), nodeB(B), rest_length(rl), stiffness(s), damping(d) {}
};

struct SurfaceTriangle
{
    int a, b, c;
};

static Vector3 triangleCentroid(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2)
{
    return Vector3Scale(Vector3Add(Vector3Add(p0, p1), p2), 1.0f / 3.0f);
}

static bool triangleGeometry(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, float &area, Vector3 &normal)
{
    Vector3 e1 = Vector3Subtract(p1, p0);
    Vector3 e2 = Vector3Subtract(p2, p0);
    Vector3 cross = Vector3CrossProduct(e1, e2);
    float area2 = Vector3Length(cross);
    if (area2 < 1e-8f)
        return false;

    area = 0.5f * area2;
    normal = Vector3Scale(cross, 1.0f / area2);
    return true;
}

static bool isPointInFluid(const Vector3 &point, const FluidEnvironment &env)
{
    return point.y <= env.waterSurfaceY && point.y >= env.waterBottomY;
}

static float computeMeshVolume(const std::vector<Node> &nodes, const std::vector<SurfaceTriangle> &tris)
{
    float volume = 0.0f;
    for (const auto &tri : tris)
    {
        Vector3 p0 = nodes[tri.a].position;
        Vector3 p1 = nodes[tri.b].position;
        Vector3 p2 = nodes[tri.c].position;
        volume += Vector3DotProduct(p0, Vector3CrossProduct(p1, p2)) / 6.0f;
    }
    return std::fabs(volume);
}

class PhysicsBody
{
protected:
    std::vector<Node> nodes;
    std::vector<Spring> springs;
    std::vector<SurfaceTriangle> surfaceTriangles;
    float restVolume = 0.0f;

    void addSpring(int a, int b, float k, float d)
    {
        Vector3 pa = nodes[a].position, pb = nodes[b].position;
        float dx = pb.x - pa.x, dy = pb.y - pa.y, dz = pb.z - pa.z;
        float rest = std::sqrt(dx * dx + dy * dy + dz * dz);
        springs.emplace_back(&nodes[a], &nodes[b], rest, k, d);
    }

    void addSurfaceTriangle(int a, int b, int c)
    {
        surfaceTriangles.push_back({a, b, c});
    }

    void finalizeBody()
    {
        restVolume = computeMeshVolume(nodes, surfaceTriangles);
    }

    Vector3 computeCenterOfMass() const
    {
        Vector3 com = Vector3Zero();
        float totalMass = 0.0f;
        for (const auto &n : nodes)
        {
            com = Vector3Add(com, Vector3Scale(n.position, (float)n.mass));
            totalMass += (float)n.mass;
        }
        if (totalMass < 1e-6f)
            return Vector3Zero();
        return Vector3Scale(com, 1.0f / totalMass);
    }

    float computeTotalMass() const
    {
        float totalMass = 0.0f;
        for (const auto &n : nodes)
            totalMass += (float)n.mass;
        return totalMass;
    }

    void applyBuoyancy(const FluidEnvironment &env)
    {
        if (restVolume <= 0.0f)
            return;

        Vector3 com = computeCenterOfMass();
        if (!isPointInFluid(com, env))
            return;

        float buoyancyMagnitude = env.fluidDensity * env.gravityMagnitude * restVolume;
        Vector3 buoyancy = Vector3Scale((Vector3){0.0f, 1.0f, 0.0f}, buoyancyMagnitude);

        float totalMass = computeTotalMass();
        for (auto &n : nodes)
            n.force = Vector3Add(n.force, Vector3Scale(buoyancy, (float)n.mass / totalMass));
    }

    void applySurfaceDrag(const FluidEnvironment &env)
    {
        for (const auto &tri : surfaceTriangles)
        {
            Node &n0 = nodes[tri.a];
            Node &n1 = nodes[tri.b];
            Node &n2 = nodes[tri.c];

            Vector3 p0 = n0.position, p1 = n1.position, p2 = n2.position;
            Vector3 v0 = n0.velocity, v1 = n1.velocity, v2 = n2.velocity;

            Vector3 center = triangleCentroid(p0, p1, p2);
            if (!isPointInFluid(center, env))
                continue;

            float area = 0.0f;
            Vector3 normal = Vector3Zero();
            if (!triangleGeometry(p0, p1, p2, area, normal))
                continue;

            Vector3 vel = Vector3Scale(Vector3Add(Vector3Add(v0, v1), v2), 1.0f / 3.0f);
            Vector3 vRel = Vector3Subtract(vel, env.fluidVelocity);

            float vn = Vector3DotProduct(vRel, normal);
            Vector3 vNormal = Vector3Scale(normal, vn);
            Vector3 vTangent = Vector3Subtract(vRel, vNormal);

            Vector3 F_drag = Vector3Subtract(
                Vector3Scale(vNormal, env.dragNormal * area * vn),
                Vector3Scale(vTangent, env.dragTangent * area));

            Vector3 third = Vector3Scale(F_drag, 1.0f / 3.0f);
            n0.force = Vector3Subtract(n0.force, third);
            n1.force = Vector3Subtract(n1.force, third);
            n2.force = Vector3Subtract(n2.force, third);
        }
    }

    void applyFluidForces(const FluidEnvironment &env)
    {
        applyBuoyancy(env);
        applySurfaceDrag(env);
    }

    void clampVelocities(float maxSpeed)
    {
        for (auto &n : nodes)
        {
            float speed = Vector3Length(n.velocity);
            if (speed > maxSpeed)
                n.velocity = Vector3Scale(n.velocity, maxSpeed / speed);
        }
    }

public:
    std::vector<Node> &getNodes() { return nodes; };
    std::vector<Spring> &getSprings() { return springs; };
    const std::vector<SurfaceTriangle> &getSurfaceTriangles() const { return surfaceTriangles; }

    void updatePhysicsImplicit(float dt, const FluidEnvironment *fluid = nullptr)
    {
        const int substeps = 4;
        float subDt = dt / substeps;
        for (int step = 0; step < substeps; step++)
            updatePhysicsImplicitStep(subDt, fluid);
    }

private:
    void updatePhysicsImplicitStep(float dt, const FluidEnvironment *fluid)
    {
        int N = nodes.size();

        Eigen::SparseMatrix<float> A(3 * N, 3 * N);
        Eigen::VectorXf rhs = Eigen::VectorXf::Zero(3 * N);

        std::vector<Eigen::Triplet<float>> triplets;

        // mass on diagonal
        for (int i = 0; i < N; i++)
            for (int k = 0; k < 3; k++)
                triplets.push_back({3 * i + k, 3 * i + k, (float)nodes[i].mass});

        for (auto &n : getNodes())
            n.force = Vector3Zero();

        for (auto &s : getSprings())
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f)
                continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            auto F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);

            Eigen::Vector3f dh(d_hat.x, d_hat.y, d_hat.z);
            Eigen::Matrix3f M_outer = dh * dh.transpose();
            Eigen::Matrix3f M_perp = Eigen::Matrix3f::Identity() - M_outer;

            Eigen::Matrix3f K_spring = -s.stiffness * M_outer - s.stiffness * (1.0f - s.rest_length / L) * M_perp;
            Eigen::Matrix3f D_spring = -s.damping * M_outer;

            int na = s.nodeA - &nodes[0];
            int nb = s.nodeB - &nodes[0];
            Eigen::Matrix3f block = dt * dt * K_spring + dt * D_spring;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    triplets.push_back({3 * na + i, 3 * na + j, -block(i, j)}); // A diagonal
                    triplets.push_back({3 * nb + i, 3 * nb + j, -block(i, j)}); // B diagonal
                    triplets.push_back({3 * na + i, 3 * nb + j, block(i, j)});  // off-diagonal AB
                    triplets.push_back({3 * nb + i, 3 * na + j, block(i, j)});  // off-diagonal BA
                };

            Eigen::Vector3f va(nodes[na].velocity.x, nodes[na].velocity.y, nodes[na].velocity.z);
            Eigen::Vector3f vb(nodes[nb].velocity.x, nodes[nb].velocity.y, nodes[nb].velocity.z);

            Eigen::Vector3f contrib_a = dt * dt * (K_spring * vb - K_spring * va);
            Eigen::Vector3f contrib_b = dt * dt * (K_spring * va - K_spring * vb);

            rhs.segment<3>(3 * na) += contrib_a;
            rhs.segment<3>(3 * nb) += contrib_b;
        };

        if (fluid)
            applyFluidForces(*fluid);

        for (auto &n : getNodes())
            n.force = Vector3Add(n.force, Vector3Scale(GLOBAL_GRAVITY, n.mass));

        A.setFromTriplets(triplets.begin(), triplets.end());

        for (int i = 0; i < N; i++)
        {
            rhs(3 * i + 0) += dt * nodes[i].force.x;
            rhs(3 * i + 1) += dt * nodes[i].force.y;
            rhs(3 * i + 2) += dt * nodes[i].force.z;
        };

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
        solver.compute(A);
        Eigen::VectorXf dv = solver.solve(rhs);

        for (int i = 0; i < N; i++)
        {
            nodes[i].velocity.x += dv(3 * i + 0);
            nodes[i].velocity.y += dv(3 * i + 1);
            nodes[i].velocity.z += dv(3 * i + 2);
            nodes[i].position.x += dt * nodes[i].velocity.x;
            nodes[i].position.y += dt * nodes[i].velocity.y;
            nodes[i].position.z += dt * nodes[i].velocity.z;
        }

        clampVelocities(12.0f);

        const float floorY = -5.0f;
        for (auto &n : getNodes())
        {
            if (n.position.y < floorY)
            {
                n.position.y = floorY;
                if (n.velocity.y < 0.0f)
                    n.velocity.y = -n.velocity.y * 0.4f;
            }
        }
    };
public:
    void updatePhysicsExplicit(float dt, const FluidEnvironment *fluid = nullptr)
    {
        for (auto &n : getNodes())
            n.force = Vector3Zero();

        for (auto &s : getSprings())
        {
            Vector3 d = Vector3Subtract(s.nodeB->position, s.nodeA->position);
            float L = Vector3Length(d);
            if (L < 1e-6f)
                continue;
            Vector3 d_hat = Vector3Normalize(d);
            float f_s = s.stiffness * (L - s.rest_length);
            float v_rel = Vector3DotProduct(Vector3Subtract(s.nodeB->velocity, s.nodeA->velocity), d_hat);
            float f_d = s.damping * v_rel;
            auto F = Vector3Scale(d_hat, -(f_s + f_d));
            s.nodeA->force = Vector3Subtract(s.nodeA->force, F);
            s.nodeB->force = Vector3Add(s.nodeB->force, F);
        };

        if (fluid)
            applyFluidForces(*fluid);

        for (auto &n : getNodes())
        {
            n.force = Vector3Add(n.force, Vector3Scale(GLOBAL_GRAVITY, n.mass));
            n.acceleration = Vector3Scale(n.force, 1.0f / n.mass);
            n.velocity = Vector3Add(n.velocity, Vector3Scale(n.acceleration, dt));
            n.position = Vector3Add(n.position, Vector3Scale(n.velocity, dt));
        };

        clampVelocities(12.0f);

        // Plane collision at y = -5.0f
        const float floorY = -5.0f;
        for (auto &n : getNodes())
        {
            if (n.position.y < floorY)
            {
                n.position.y = floorY; // push node back above floor
                if (n.velocity.y < 0.0f)
                    n.velocity.y = -n.velocity.y * 0.4f; // e.g. restitution = 0.4f
            }
        }
    };
};


class Cube : public PhysicsBody
{
    float size;

public:
    Cube(float size = 1.0f, float stiffness = 100.0f, float damping = 5.0f) : size(size)
    {
        float h = size * 0.5f;
        nodes.reserve(8);

        // 8 corners: back face (z = -h), then front face (z = +h)
        nodes.push_back(Node({-h, -h, -h})); // 0 back-bottom-left
        nodes.push_back(Node({h, -h, -h}));  // 1 back-bottom-right
        nodes.push_back(Node({-h, h, -h}));  // 2 back-top-left
        nodes.push_back(Node({h, h, -h}));   // 3 back-top-right
        nodes.push_back(Node({-h, -h, h}));  // 4 front-bottom-left
        nodes.push_back(Node({h, -h, h}));   // 5 front-bottom-right
        nodes.push_back(Node({-h, h, h}));   // 6 front-top-left
        nodes.push_back(Node({h, h, h}));    // 7 front-top-right

        // 12 outer edge springs only (no face diagonals or body diagonals)
        addSpring(0, 1, stiffness, damping); // back bottom
        addSpring(1, 3, stiffness, damping); // back right
        addSpring(3, 2, stiffness, damping); // back top
        addSpring(2, 0, stiffness, damping); // back left

        addSpring(4, 5, stiffness, damping); // front bottom
        addSpring(5, 7, stiffness, damping); // front right
        addSpring(7, 6, stiffness, damping); // front top
        addSpring(6, 4, stiffness, damping); // front left

        addSpring(0, 4, stiffness, damping); // bottom left
        addSpring(1, 5, stiffness, damping); // bottom right
        addSpring(2, 6, stiffness, damping); // top left
        addSpring(3, 7, stiffness, damping); // top right

        // Face diagonals on outer surfaces only — resist face collapse/inflation
        float faceK = stiffness * 0.75f;
        addSpring(0, 3, faceK, damping); // back
        addSpring(1, 2, faceK, damping);
        addSpring(4, 7, faceK, damping); // front
        addSpring(5, 6, faceK, damping);
        addSpring(0, 5, faceK, damping); // bottom
        addSpring(1, 4, faceK, damping);
        addSpring(2, 7, faceK, damping); // top
        addSpring(3, 6, faceK, damping);
        addSpring(0, 6, faceK, damping); // left
        addSpring(2, 4, faceK, damping);
        addSpring(1, 7, faceK, damping); // right
        addSpring(3, 5, faceK, damping);

        buildSurfaceMesh();
        restVolume = size * size * size;
    }

private:
    void buildSurfaceMesh()
    {
        // Bottom (y = -h), normal -Y
        addSurfaceTriangle(0, 5, 1);
        addSurfaceTriangle(0, 4, 5);
        // Top (y = +h), normal +Y
        addSurfaceTriangle(2, 3, 7);
        addSurfaceTriangle(2, 7, 6);
        // Back (z = -h), normal -Z
        addSurfaceTriangle(0, 1, 3);
        addSurfaceTriangle(0, 3, 2);
        // Front (z = +h), normal +Z
        addSurfaceTriangle(4, 5, 7);
        addSurfaceTriangle(4, 7, 6);
        // Left (x = -h), normal -X
        addSurfaceTriangle(0, 2, 6);
        addSurfaceTriangle(0, 6, 4);
        // Right (x = +h), normal +X
        addSurfaceTriangle(1, 3, 7);
        addSurfaceTriangle(1, 7, 5);
    }
};

int main()
{
    FluidEnvironment fluid;
    Cube cube;
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "Fish Sim");
    Camera3D camera = {{5, 5, 5}, {0, 0, 0}, {0, 1, 0}, 45.0f, CAMERA_PERSPECTIVE};

    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = std::fminf(GetFrameTime(), 1.0f / 60.0f);
        cube.updatePhysicsImplicit(dt, &fluid);

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);

        DrawPlane((Vector3){0.0f, fluid.waterSurfaceY, 0.0f}, (Vector2){32.0f, 32.0f}, (Color){80, 140, 220, 80});
        DrawPlane((Vector3){0.0f, -5.0f, 0.0f}, (Vector2){32.0f, 32.0f}, ORANGE);

        for (auto &n : cube.getNodes())
            DrawSphere(n.position, 0.05f, GREEN);
        for (auto &s : cube.getSprings())
            DrawLine3D(s.nodeA->position, s.nodeB->position, WHITE);
        for (const auto &tri : cube.getSurfaceTriangles())
        {
            const auto &nodes = cube.getNodes();
            DrawTriangle3D(nodes[tri.a].position, nodes[tri.b].position, nodes[tri.c].position, (Color){0, 255, 120, 40});
        }

        EndMode3D();
        EndDrawing();
    };

    CloseWindow();
    return 0;
};
