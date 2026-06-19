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

#pragma once

// Serialization helpers used to record the full kinematic state of the
// simulation each step. The output is a deterministic, plain-text CSV that a
// port of this solver can diff against to verify it behaves identically.
//
// This header has no rendering or platform dependencies, so it works in the
// AVBD_HEADLESS configuration (see solver.h).

#include <cstdio>
#include <vector>

#include "solver.h"

// Returns the bodies of the solver in *creation order*, i.e. index 0 is the
// first body created by the scene. The solver stores bodies in a singly linked
// list that is prepended to on creation, so the raw list order is the reverse
// of creation order; we reverse it here so indices are stable and intuitive and
// match the order a port would get by appending bodies to an array as it builds
// the same scene.
inline std::vector<Rigid*> bodiesInCreationOrder(Solver* solver)
{
    std::vector<Rigid*> result;
    for (Rigid* b = solver->bodies; b != 0; b = b->next)
        result.push_back(b);
    std::vector<Rigid*> ordered(result.rbegin(), result.rend());
    return ordered;
}

// Writes the metadata block describing the solver configuration used to produce
// the data. Lines are prefixed with '#' so CSV consumers can skip them. Floats
// are printed with %.9g, which round-trips a 32-bit float exactly.
inline void recordMetadata(FILE* f, Solver* solver, const char* sceneName, int steps)
{
    fprintf(f, "# avbd-demo2d reference data\n");
    fprintf(f, "# scene=%s\n", sceneName);
    fprintf(f, "# steps=%d\n", steps);
    fprintf(f, "# bodies=%zu\n", bodiesInCreationOrder(solver).size());
    fprintf(f, "# dt=%.9g\n", solver->dt);
    fprintf(f, "# gravity=%.9g\n", solver->gravity);
    fprintf(f, "# iterations=%d\n", solver->iterations);
    fprintf(f, "# alpha=%.9g\n", solver->alpha);
    fprintf(f, "# beta=%.9g\n", solver->beta);
    fprintf(f, "# gamma=%.9g\n", solver->gamma);
    fprintf(f, "# postStabilize=%d\n", solver->postStabilize ? 1 : 0);
}

// Writes the CSV column header.
inline void recordHeader(FILE* f)
{
    fprintf(f, "step,body,px,py,angle,vx,vy,omega\n");
}

// Writes one CSV row per body for the given step. Columns:
//   step   simulation step index (0 = initial state before any step)
//   body   body index in creation order (see bodiesInCreationOrder)
//   px,py  body centre-of-mass position
//   angle  body orientation in radians
//   vx,vy  linear velocity
//   omega  angular velocity
inline void recordStep(FILE* f, Solver* solver, int step)
{
    std::vector<Rigid*> bodies = bodiesInCreationOrder(solver);
    for (size_t i = 0; i < bodies.size(); i++)
    {
        Rigid* b = bodies[i];
        fprintf(f, "%d,%zu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            step, i,
            b->position.x, b->position.y, b->position.z,
            b->velocity.x, b->velocity.y, b->velocity.z);
    }
}
