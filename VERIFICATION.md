# Reference Data & Cross-Engine Verification

This document explains how to generate **golden reference data** from this
AVBD demo and how to consume it to verify that a port of the solver (to another
language or engine) reproduces the simulation exactly.

The simulation is fully deterministic: given the same scene, the same solver
parameters, and the same sequence of floating-point operations, every run
produces bit-for-bit identical results. The reference data captures the complete
kinematic state of every body at every step, so a port can step the same scene
and diff its trajectory against the reference.

---

## 1. What the data contains

The generator runs a chosen scene headless (no rendering) and writes one CSV
file. The file has two parts:

### Metadata header (comment lines, prefixed with `#`)

```
# avbd-demo2d reference data
# scene=Pyramid
# steps=120
# bodies=211
# dt=0.0166666675
# gravity=-10
# iterations=10
# alpha=0.99000001
# beta=100000
# gamma=0.99000001
# postStabilize=1
```

These are the solver parameters used to produce the data (`Solver::defaultParams`).
Your port **must** use the same values or the trajectories will not match. Any
line starting with `#` should be skipped by a CSV parser.

### Per-step body state (CSV)

```
step,body,px,py,angle,vx,vy,omega
0,0,0,-2,0,0,0,0
0,1,-10,0,0,0,0,0
...
1,0,0,-2,0,0,0,0
...
```

| Column  | Meaning                                                        |
|---------|----------------------------------------------------------------|
| `step`  | Step index. `0` is the initial state **before** any `step()`.  |
| `body`  | Body index in **creation order** (see below).                  |
| `px,py` | Centre-of-mass position.                                       |
| `angle` | Orientation in radians.                                        |
| `vx,vy` | Linear velocity.                                               |
| `omega` | Angular velocity.                                             |

There is one row per body per step. For `S` steps and `B` bodies the file has
`(S + 1) * B` data rows (step `0` plus steps `1..S`).

**Body ordering.** Bodies are indexed in the order the scene **creates** them
(index `0` is the first `new Rigid(...)` in the scene function). Internally the
solver stores bodies in a linked list that is prepended to on creation, so the
generator reverses that list to recover creation order. A port that builds the
same scene by appending bodies to an array will naturally match this ordering.

**Precision.** All floats are printed with `%.9g`, which round-trips a 32-bit
IEEE-754 `float` exactly. If you parse these values back into 32-bit floats you
recover the identical bit pattern.

---

## 2. Generating the data

> **Pre-generated data.** A full set of reference files (every scene, 120 steps)
> is checked in under [`reference/`](reference/), one `<scene>.csv` per scene.
> You can use those directly, or regenerate them as below. They were produced on
> a single toolchain, so use them for tolerance-based comparison unless you build
> the generator with the same compiler/CPU (see "Exact vs. approximate matching"
> below). To regenerate the whole set: `avbd_generate --all reference --steps 120`.

### Build the headless tools

The generator and tests build **without** SDL/ImGui/OpenGL (they compile the
solver core with `AVBD_HEADLESS` defined). With CMake:

```
mkdir build
cd build
cmake ..
cmake --build . --config Release --target avbd_generate
cmake --build . --config Release --target avbd_tests
```

Or compile directly with any C++17 compiler (no submodules needed):

```
g++ -std=c++17 -O2 -DAVBD_HEADLESS -Isource \
    tools/generate.cpp \
    source/collide.cpp source/force.cpp source/joint.cpp source/manifold.cpp \
    source/motor.cpp source/rigid.cpp source/solver.cpp source/spring.cpp \
    -o avbd_generate
```

### Run it

```
# List the available scenes (name, index, slug)
avbd_generate --list

# Generate 120 steps of the Pyramid scene to stdout
avbd_generate --scene pyramid --steps 120

# Write a single scene to a file
avbd_generate --scene dynamic_friction --steps 240 --out friction.csv

# Generate every scene into a directory (one <slug>.csv per scene)
mkdir reference
avbd_generate --all reference --steps 120
```

Options:

| Option                  | Default        | Description                                  |
|-------------------------|----------------|----------------------------------------------|
| `--scene <name\|index>` | `4` (Pyramid)  | Scene to run. Name match is case/spacing-insensitive (`"Dynamic Friction"`, `dynamic_friction`). |
| `--steps <N>`           | `120`          | Number of steps to simulate.                 |
| `--out <file>`          | stdout         | Output CSV path.                             |
| `--all <dir>`           | —              | Write every scene into `<dir>/<slug>.csv`.   |
| `--list`                | —              | List scenes and exit.                        |

---

## 3. Consuming the data in your port

The workflow to verify a port:

1. **Reproduce the scene.** Build the identical scene in your port: same bodies,
   same sizes, densities, positions, velocities, and constraints, created in the
   same order. The scene definitions live in [`source/scenes.h`](source/scenes.h).
2. **Match the solver parameters.** Read the `#` metadata and configure your
   solver: `dt`, `gravity`, `iterations`, `alpha`, `beta`, `gamma`,
   `postStabilize`.
3. **Step and compare.** For `step` from `1` to `N`, advance your solver one
   step and compare every body's `(px, py, angle, vx, vy, omega)` against the
   reference row for that `(step, body)`.

### Exact vs. approximate matching

- **Same toolchain / platform as the reference:** results are typically
  **bit-for-bit** identical. You can compare the raw float bits.
- **Different language, compiler, CPU, or math library:** floating-point
  results can differ in the last bits and **drift** over many steps (especially
  in stiff, contact-heavy scenes). This is expected — it does not mean the port
  is wrong. Compare with a tolerance.

Recommended starting tolerances (absolute) for a float32 port:

| Quantity            | Tolerance     |
|---------------------|---------------|
| position / angle    | `1e-3`        |
| velocity / omega    | `1e-2`        |

Because error accumulates, the most informative check is **per-step divergence**:
diff step `1` first. If step `1` matches tightly but step `500` has drifted, the
port logic is correct and you are only seeing float divergence. If step `1`
already disagrees beyond tolerance, you have a real bug — and the small step
count makes it easy to localize (which body, which constraint).

> Tip: start with simple, near-deterministic scenes — `Ground`, `Spring`,
> `Rod`, `Rope`, single free-falling boxes — before the stiff stacking scenes
> like `Pyramid` or `Cards`, where float divergence appears soonest.

### Minimal consumer pseudocode

```python
meta, rows = parse_csv("pyramid.csv")        # rows keyed by (step, body)
solver = build_scene("pyramid")              # same bodies, same order
solver.configure(meta)                       # dt, gravity, iterations, ...

POS_TOL, VEL_TOL = 1e-3, 1e-2
for step in range(1, meta["steps"] + 1):
    solver.step()
    for body_index, body in enumerate(solver.bodies_in_creation_order()):
        ref = rows[(step, body_index)]
        assert abs(body.px - ref.px)       <= POS_TOL
        assert abs(body.py - ref.py)       <= POS_TOL
        assert abs(body.angle - ref.angle) <= POS_TOL
        assert abs(body.vx - ref.vx)       <= VEL_TOL
        assert abs(body.vy - ref.vy)       <= VEL_TOL
        assert abs(body.omega - ref.omega) <= VEL_TOL
```

---

## 4. Running the unit tests

The unit tests cover the math library, rigid-body mass properties, the
integrator, collision detection, and — importantly for this workflow — that the
simulation is deterministic across runs.

```
cd build
cmake --build . --config Release --target avbd_tests
ctest -C Release --output-on-failure
```

Or run the executable directly. It exits non-zero if any check fails:

```
./avbd_tests
```

Directly with a compiler:

```
g++ -std=c++17 -O2 -DAVBD_HEADLESS -Isource \
    tests/tests.cpp \
    source/collide.cpp source/force.cpp source/joint.cpp source/manifold.cpp \
    source/motor.cpp source/rigid.cpp source/solver.cpp source/spring.cpp \
    -o avbd_tests && ./avbd_tests
```

---

## 5. Notes & caveats

- **Determinism is required for this to work.** The solver uses no randomness
  and no wall-clock time, so a given build is reproducible. The `test_determinism`
  unit test asserts this by running a scene twice and requiring identical state.
- **Contacts are transient, body state is not.** The set of active contact
  constraints changes every step, but the bodies themselves persist for the life
  of a scene. The reference data therefore records body state, which is the
  authoritative ground truth: if every body's position and velocity matches at
  every step, the whole simulation matches.
- **Do not change the parameters mid-comparison.** The interactive demo lets you
  edit `dt`, `gravity`, iterations, etc. The generator always uses
  `Solver::defaultParams()`; keep your port on the values in the metadata header.
