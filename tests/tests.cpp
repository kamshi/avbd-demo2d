/*
* Copyright (c) 2025 Chris Giles
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Chris Giles makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

// Unit tests for the AVBD 2D solver core.
//
// Built headless (AVBD_HEADLESS) so there are no SDL/OpenGL dependencies. Uses a
// tiny assertion framework rather than an external test library, to match the
// minimal-dependency style of the rest of the project. Returns a non-zero exit
// code if any check fails, so it plugs straight into CTest.

#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

#include "maths.h"
#include "solver.h"
#include "scenes.h"
#include "record.h"

// ---------------------------------------------------------------------------
// Minimal test framework
// ---------------------------------------------------------------------------

static int g_checks = 0;
static int g_failures = 0;
static const char* g_currentTest = "";

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("  FAIL [%s] %s:%d: %s\n", g_currentTest, __FILE__,         \
                   __LINE__, #cond);                                           \
        }                                                                      \
    } while (0)

// Absolute-or-relative tolerance comparison suitable for 32-bit float math.
static bool approx(float a, float b, float eps = 1e-4f)
{
    float d = fabsf(a - b);
    if (d <= eps)
        return true;
    float scale = max(fabsf(a), fabsf(b));
    return d <= eps * scale;
}

#define CHECK_NEAR(a, b, eps)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        float _va = (a), _vb = (b);                                            \
        if (!approx(_va, _vb, eps)) {                                          \
            g_failures++;                                                      \
            printf("  FAIL [%s] %s:%d: %s (%.9g) ~= %s (%.9g)\n",              \
                   g_currentTest, __FILE__, __LINE__, #a, _va, #b, _vb);       \
        }                                                                      \
    } while (0)

#define RUN(test)                                                             \
    do {                                                                       \
        g_currentTest = #test;                                                 \
        printf("- %s\n", #test);                                               \
        test();                                                                \
    } while (0)

static const float PI = 3.14159265358979f;

// ---------------------------------------------------------------------------
// maths.h
// ---------------------------------------------------------------------------

static void test_vector_ops()
{
    CHECK_NEAR(dot(float2{ 1, 2 }, float2{ 3, 4 }), 11.0f, 1e-5f);
    CHECK_NEAR(dot(float3{ 1, 2, 3 }, float3{ 4, 5, 6 }), 32.0f, 1e-5f);
    CHECK_NEAR(cross(float2{ 1, 0 }, float2{ 0, 1 }), 1.0f, 1e-5f);
    CHECK_NEAR(cross(float2{ 0, 1 }, float2{ 1, 0 }), -1.0f, 1e-5f);
    CHECK_NEAR(length(float2{ 3, 4 }), 5.0f, 1e-5f);
    CHECK_NEAR(lengthSq(float2{ 3, 4 }), 25.0f, 1e-5f);
    CHECK_NEAR(length(float3{ 2, 3, 6 }), 7.0f, 1e-5f);

    // Operators
    float2 a = float2{ 1, 2 } + float2{ 3, 4 };
    CHECK_NEAR(a.x, 4.0f, 1e-5f);
    CHECK_NEAR(a.y, 6.0f, 1e-5f);
    float3 b = float3{ 1, 2, 3 } * 2.0f;
    CHECK_NEAR(b.z, 6.0f, 1e-5f);
}

static void test_scalar_ops()
{
    CHECK_NEAR(clamp(5.0f, 0.0f, 1.0f), 1.0f, 1e-6f);
    CHECK_NEAR(clamp(-5.0f, 0.0f, 1.0f), 0.0f, 1e-6f);
    CHECK_NEAR(clamp(0.5f, 0.0f, 1.0f), 0.5f, 1e-6f);
    CHECK_NEAR(min(3.0f, 7.0f), 3.0f, 1e-6f);
    CHECK_NEAR(max(3.0f, 7.0f), 7.0f, 1e-6f);
    CHECK_NEAR(sign(-2.0f), -1.0f, 1e-6f);
    CHECK_NEAR(sign(2.0f), 1.0f, 1e-6f);
    CHECK_NEAR(sign(0.0f), 0.0f, 1e-6f);
}

static void test_rotation_and_transform()
{
    // Rotating (1,0) by 90 degrees gives (0,1)
    float2 r = rotation(PI / 2.0f) * float2{ 1, 0 };
    CHECK_NEAR(r.x, 0.0f, 1e-5f);
    CHECK_NEAR(r.y, 1.0f, 1e-5f);

    // Rotation matrix is orthonormal: R * R^T = I
    float2x2 R = rotation(0.7f);
    float2x2 I = R * transpose(R);
    CHECK_NEAR(I[0][0], 1.0f, 1e-5f);
    CHECK_NEAR(I[0][1], 0.0f, 1e-5f);
    CHECK_NEAR(I[1][0], 0.0f, 1e-5f);
    CHECK_NEAR(I[1][1], 1.0f, 1e-5f);

    // transform applies rotation then translation
    float3 q = float3{ 10, 20, PI / 2.0f };
    float2 t = transform(q, float2{ 1, 0 });
    CHECK_NEAR(t.x, 10.0f, 1e-4f);
    CHECK_NEAR(t.y, 21.0f, 1e-4f);
}

static void test_matrix_ops()
{
    // outer product: outer(a,b)[i][j] = a[i]*b[j]
    float3x3 O = outer(float3{ 1, 2, 3 }, float3{ 4, 5, 6 });
    CHECK_NEAR(O[0][0], 4.0f, 1e-5f);
    CHECK_NEAR(O[2][2], 18.0f, 1e-5f);
    CHECK_NEAR(O[1][2], 12.0f, 1e-5f);

    float3x3 D = diagonal(2, 3, 4);
    float3 d = D * float3{ 1, 1, 1 };
    CHECK_NEAR(d.x, 2.0f, 1e-5f);
    CHECK_NEAR(d.y, 3.0f, 1e-5f);
    CHECK_NEAR(d.z, 4.0f, 1e-5f);
}

static void test_solve_ldlt()
{
    // Symmetric positive-definite system; verify the LDL^T solver recovers x.
    float3x3 A = {
        4, 1, 0,
        1, 3, 1,
        0, 1, 2
    };
    float3 x = { 1, 2, 3 };
    float3 b = A * x;               // b = {6, 10, 8}
    float3 xs = solve(A, b);
    CHECK_NEAR(xs.x, 1.0f, 1e-4f);
    CHECK_NEAR(xs.y, 2.0f, 1e-4f);
    CHECK_NEAR(xs.z, 3.0f, 1e-4f);

    // Diagonal system is a trivial special case.
    float3x3 Ad = diagonal(2, 4, 8);
    float3 xd = solve(Ad, float3{ 2, 4, 8 });
    CHECK_NEAR(xd.x, 1.0f, 1e-5f);
    CHECK_NEAR(xd.y, 1.0f, 1e-5f);
    CHECK_NEAR(xd.z, 1.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// Rigid body
// ---------------------------------------------------------------------------

static void test_rigid_mass_properties()
{
    Solver solver;

    // Dynamic body: mass = w*h*density, moment = mass*(w^2+h^2)/12
    Rigid* body = new Rigid(&solver, { 2, 1 }, 3.0f, 0.5f, { 0, 0, 0 });
    CHECK_NEAR(body->mass, 6.0f, 1e-5f);
    CHECK_NEAR(body->moment, 6.0f * (4.0f + 1.0f) / 12.0f, 1e-5f);
    CHECK_NEAR(body->radius, length(float2{ 1.0f, 0.5f }), 1e-5f);

    // Density 0 means a static / kinematic body (mass 0).
    Rigid* stat = new Rigid(&solver, { 100, 1 }, 0.0f, 0.5f, { 0, 0, 0 });
    CHECK_NEAR(stat->mass, 0.0f, 1e-6f);
    CHECK_NEAR(stat->moment, 0.0f, 1e-6f);
}

static void test_constrained_to()
{
    Solver solver;
    Rigid* a = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 0, 0, 0 });
    Rigid* b = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 5, 0, 0 });
    Rigid* c = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 10, 0, 0 });

    CHECK(!a->constrainedTo(b));
    new Joint(&solver, a, b, { 0, 0 }, { 0, 0 });
    CHECK(a->constrainedTo(b));
    CHECK(b->constrainedTo(a));
    CHECK(!a->constrainedTo(c));
}

// ---------------------------------------------------------------------------
// Integration
// ---------------------------------------------------------------------------

// A single free body (no contacts, no constraints) integrates with symplectic
// (semi-implicit) Euler exactly:
//   v_n = v0 + n*g*dt
//   p_n = p0 + n*v0*dt + g*dt^2 * n(n+1)/2
//
// Note on tolerances: the solver recovers velocity as (position - initial)/dt.
// When the body sits at a large coordinate, that subtraction cancels two
// near-equal float32 values and the divide by dt amplifies the leftover rounding
// error to ~ulp(coordinate)/dt. This is inherent to float32, not a solver bug,
// so the tolerance below scales with the coordinate magnitude. We deliberately
// test at a large height (y=100) as well as near the origin to exercise this.
static void test_free_fall_integration()
{
    Solver probe;                  // just to read default dt / gravity
    float dt = probe.dt;
    float g = probe.gravity;

    struct Case { float3 p0; float3 v0; };
    Case cases[] = {
        { { 0,   0, 0 }, { 2, 0, 0 } },   // near origin: essentially exact
        { { 0, 100, 0 }, { 2, 0, 0 } },   // large height: float32 cancellation
    };

    for (Case c : cases)
    {
        // float32 rounding floor at this coordinate magnitude (~2^-23 relative).
        float mag = max(fabsf(c.p0.y), 1.0f);
        float ulp = mag * 1.2e-7f;
        float posTol = max(1e-4f, 4.0f * ulp);
        float velTol = max(1e-4f, 8.0f * ulp / dt);

        for (int n : { 1, 10, 30 })
        {
            Solver solver;
            Rigid* body = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, c.p0, c.v0);
            for (int i = 0; i < n; i++)
                solver.step();

            float expectedVy = c.v0.y + n * g * dt;
            float expectedPy = c.p0.y + n * c.v0.y * dt + g * dt * dt * (n * (n + 1) / 2.0f);
            float expectedPx = c.p0.x + n * c.v0.x * dt;

            CHECK_NEAR(body->velocity.y, expectedVy, velTol);
            CHECK_NEAR(body->velocity.x, c.v0.x, velTol);
            CHECK_NEAR(body->position.y, expectedPy, posTol);
            CHECK_NEAR(body->position.x, expectedPx, posTol);
        }
    }
}

// A box dropped onto static ground should come to rest on top of it and stay
// there (not fall through, not explode).
static void test_resting_on_ground()
{
    Solver solver;
    new Rigid(&solver, { 100, 1 }, 0.0f, 0.5f, { 0, 0, 0 });   // ground, top at y=0.5
    Rigid* box = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 0, 1, 0 }); // resting

    for (int i = 0; i < 120; i++)
        solver.step();

    CHECK(isfinite(box->position.y));
    // Bottom of the box sits at box.y - 0.5; should rest near the ground top 0.5.
    CHECK(box->position.y > 0.9f);
    CHECK(box->position.y < 1.1f);
}

// ---------------------------------------------------------------------------
// Collision detection
// ---------------------------------------------------------------------------

static void test_collision_detection()
{
    Solver solver;
    Rigid* a = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 0, 0, 0 });

    // Overlapping box: face-face overlap yields two contact points.
    Rigid* b = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 0.5f, 0, 0 });
    Manifold::Contact contacts[2];
    int n = Manifold::collide(a, b, contacts);
    CHECK(n == 2);

    // Clearly separated boxes: no contacts.
    Rigid* c = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 10, 0, 0 });
    int m = Manifold::collide(a, c, contacts);
    CHECK(m == 0);
}

// ---------------------------------------------------------------------------
// Spring
// ---------------------------------------------------------------------------

static void test_spring_rest_length()
{
    Solver solver;
    Rigid* anchor = new Rigid(&solver, { 1, 1 }, 0.0f, 0.5f, { 0, 0, 0 });
    Rigid* block = new Rigid(&solver, { 1, 1 }, 1.0f, 0.5f, { 0, -8, 0 });

    // rest = -1 means "use the current separation as the rest length".
    Spring* spring = new Spring(&solver, anchor, block, { 0, 0 }, { 0, 0 }, 100.0f, -1.0f);
    CHECK_NEAR(spring->rest, 8.0f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Determinism (the property the verification workflow relies on)
// ---------------------------------------------------------------------------

struct Snapshot { std::vector<float3> pos, vel; };

static Snapshot runScene(int sceneIndex, int steps)
{
    Solver solver;
    scenes[sceneIndex](&solver);
    for (int i = 0; i < steps; i++)
        solver.step();

    Snapshot s;
    for (Rigid* b : bodiesInCreationOrder(&solver))
    {
        s.pos.push_back(b->position);
        s.vel.push_back(b->velocity);
    }
    return s;
}

static void test_determinism()
{
    // Pyramid exercises many contacts; run it twice and require bit-for-bit
    // identical state. Any divergence would break cross-engine verification.
    Snapshot a = runScene(4, 40);
    Snapshot b = runScene(4, 40);

    CHECK(a.pos.size() == b.pos.size());
    CHECK(a.pos.size() > 0);

    bool identical = a.pos.size() == b.pos.size();
    for (size_t i = 0; identical && i < a.pos.size(); i++)
    {
        if (memcmp(&a.pos[i], &b.pos[i], sizeof(float3)) != 0 ||
            memcmp(&a.vel[i], &b.vel[i], sizeof(float3)) != 0)
            identical = false;
    }
    CHECK(identical);
}

static void test_all_scenes_run()
{
    // Smoke test: every registered scene loads and steps without producing
    // non-finite state.
    for (int s = 0; s < sceneCount; s++)
    {
        Solver solver;
        scenes[s](&solver);
        for (int i = 0; i < 10; i++)
            solver.step();

        bool finite = true;
        for (Rigid* b : bodiesInCreationOrder(&solver))
            if (!isfinite(b->position.x) || !isfinite(b->position.y) || !isfinite(b->position.z))
                finite = false;
        CHECK(finite);
    }
}

// ---------------------------------------------------------------------------

int main()
{
    printf("Running avbd-demo2d unit tests\n");

    RUN(test_vector_ops);
    RUN(test_scalar_ops);
    RUN(test_rotation_and_transform);
    RUN(test_matrix_ops);
    RUN(test_solve_ldlt);
    RUN(test_rigid_mass_properties);
    RUN(test_constrained_to);
    RUN(test_free_fall_integration);
    RUN(test_resting_on_ground);
    RUN(test_collision_detection);
    RUN(test_spring_rest_length);
    RUN(test_determinism);
    RUN(test_all_scenes_run);

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0)
        printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
