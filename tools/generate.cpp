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

// Headless reference-data generator.
//
// Runs one (or all) of the built-in scenes through the AVBD solver and writes
// the full per-step body state to a CSV file. The output is fully deterministic
// and is intended to be used as a golden reference when porting this solver to
// another language or engine: run the same scene in the port and diff the
// resulting state trajectories.
//
// See VERIFICATION.md for the data format and the verification workflow.

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

#include "solver.h"
#include "scenes.h"
#include "record.h"

// Convert a scene display name ("Dynamic Friction") into a filename-friendly
// slug ("dynamic_friction").
static std::string slug(const char* name)
{
    std::string s;
    for (const char* p = name; *p; p++)
    {
        char c = *p;
        if (c == ' ')
            s += '_';
        else if (c >= 'A' && c <= 'Z')
            s += (char)(c - 'A' + 'a');
        else
            s += c;
    }
    return s;
}

// Resolve a scene argument that is either a numeric index or a (case- and
// space/underscore-insensitive) scene name. Returns -1 if not found.
static int findScene(const char* arg)
{
    // Numeric index?
    bool numeric = arg[0] != '\0';
    for (const char* p = arg; *p; p++)
        if (*p < '0' || *p > '9')
            numeric = false;
    if (numeric)
    {
        int idx = atoi(arg);
        if (idx >= 0 && idx < sceneCount)
            return idx;
        return -1;
    }

    // Name match: compare slugs so "Dynamic Friction", "dynamic_friction" and
    // "dynamic friction" all match.
    std::string want = slug(arg);
    for (int i = 0; i < sceneCount; i++)
        if (slug(sceneNames[i]) == want)
            return i;
    return -1;
}

// Run a single scene for `steps` steps and write the CSV to `f`.
static void generate(FILE* f, int sceneIndex, int steps)
{
    Solver solver;
    scenes[sceneIndex](&solver);

    recordMetadata(f, &solver, sceneNames[sceneIndex], steps);
    recordHeader(f);

    // Step 0 is the initial state before any integration.
    recordStep(f, &solver, 0);
    for (int s = 1; s <= steps; s++)
    {
        solver.step();
        recordStep(f, &solver, s);
    }
}

static void usage(const char* exe)
{
    printf("Usage: %s [options]\n", exe);
    printf("  --scene <name|index>   Scene to simulate (default: 4 / Pyramid)\n");
    printf("  --steps <N>            Number of steps to simulate (default: 120)\n");
    printf("  --out <file>           Output CSV file (default: stdout)\n");
    printf("  --all <dir>            Generate every scene into <dir>/<scene>.csv\n");
    printf("  --list                 List available scenes and exit\n");
    printf("  --help                 Show this help and exit\n");
}

int main(int argc, char* argv[])
{
    int sceneIndex = 4;          // Pyramid: a good default exercising contacts
    int steps = 120;             // 2 seconds at the default 60 Hz timestep
    const char* outPath = 0;
    const char* allDir = 0;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--list") == 0)
        {
            for (int s = 0; s < sceneCount; s++)
                printf("%2d  %-18s (%s)\n", s, sceneNames[s], slug(sceneNames[s]).c_str());
            return 0;
        }
        else if (strcmp(argv[i], "--help") == 0)
        {
            usage(argv[0]);
            return 0;
        }
        else if (strcmp(argv[i], "--scene") == 0 && i + 1 < argc)
        {
            sceneIndex = findScene(argv[++i]);
            if (sceneIndex < 0)
            {
                fprintf(stderr, "Unknown scene '%s' (use --list)\n", argv[i]);
                return 1;
            }
        }
        else if (strcmp(argv[i], "--steps") == 0 && i + 1 < argc)
        {
            steps = atoi(argv[++i]);
            if (steps < 0)
            {
                fprintf(stderr, "Invalid step count\n");
                return 1;
            }
        }
        else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc)
        {
            outPath = argv[++i];
        }
        else if (strcmp(argv[i], "--all") == 0 && i + 1 < argc)
        {
            allDir = argv[++i];
        }
        else
        {
            fprintf(stderr, "Unknown argument '%s'\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (allDir)
    {
        // Generate every scene into its own file.
        for (int s = 0; s < sceneCount; s++)
        {
            std::string path = std::string(allDir) + "/" + slug(sceneNames[s]) + ".csv";
            FILE* f = fopen(path.c_str(), "wb");
            if (!f)
            {
                fprintf(stderr, "Failed to open '%s' for writing\n", path.c_str());
                return 1;
            }
            generate(f, s, steps);
            fclose(f);
            printf("Wrote %s\n", path.c_str());
        }
        return 0;
    }

    FILE* f = stdout;
    if (outPath)
    {
        f = fopen(outPath, "wb");
        if (!f)
        {
            fprintf(stderr, "Failed to open '%s' for writing\n", outPath);
            return 1;
        }
    }

    generate(f, sceneIndex, steps);

    if (f != stdout)
        fclose(f);
    return 0;
}
