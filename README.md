# Micro-Life 3D: An Autonomous Ecosystem

A real-time 3D artificial-life simulation of a microscopic aquatic ecosystem rendered in C++ and [raylib](https://www.raylib.com/). Six bacterial colonies, an amoeba predator, nutrient fields, thermal gradients, and physical obstacles inhabit a cylindrical petri dish, interacting through physics-based soft-body locomotion, hierarchical behavioral state machines, and Boids-style collective dynamics. Complex ecological phenomena— foraging aggregation, predator pursuit, colony fission and fusion, and satellite group formation— emerge from simple per-agent rules operating on local sensory information rather than scripted behaviors.

**Amanda An, Jalal Ikram, Andrew Douglas, Thien Le**  
Computer Science Department, University of California, Los Angeles — CS 275 Final Project

---

## Features

- **Soft-body physics** — mass-spring bodies with flagellar propulsion and hydrodynamic drag/buoyancy
- **Autonomous agents** — behavior state machines drive wandering, food seeking, predator escape, temperature seeking, and obstacle avoidance
- **Boid swarming** — six bacterial colonies with separation, alignment, cohesion, group merging, dispersal, and pair formation
- **Ecosystem** — nutrient field tied to temperature isotherms, heat lamps, 3D obstacles, and cocci clusters
- **Experiment B reproduction** — `experiment.cpp` replicates Section VIII-B of the final report (predator-induced colony scatter and satellite group formation), with CSV metrics logging
- **Project site** — `index.html` is a static showcase page for screenshots, videos, and the final report

---

## Requirements

| Platform | Dependencies |
|----------|--------------|
| **macOS** | Xcode Command Line Tools (`xcode-select --install`), C++14 compiler |
| **Windows** | MinGW/MSYS2 or similar with `clang++`/`g++`, raylib installed and on your library path |
| **Python** (optional) | Python 3.9+ with `pandas` and `matplotlib` for plotting experiment output |

raylib **5.5.0** is bundled for macOS at `libraries/libraylib.5.5.0.dylib`. Headers live in `include/`. Eigen is vendored under `include/Eigen/` and used by the physics solver.

---

## Quick start

### macOS — interactive demo

From the project root:

```bash
./buildnew.sh
```

This compiles `main.cpp` + `src/BoidBehavior.cpp`, links against the bundled raylib dylib, and launches the simulation.

If the build script fails because of an outdated SDK path, compile manually (adjust the SDK if needed):

```bash
SDK=$(xcrun --show-sdk-path)
clang++ -std=gnu++14 -g -stdlib=libc++ \
  -isysroot "$SDK" \
  -I./include \
  main.cpp src/BoidBehavior.cpp ./libraries/libraylib.5.5.0.dylib \
  -framework IOKit -framework Cocoa -framework OpenGL \
  -o main && ./main
```

Run the binary directly if it is already built:

```bash
./main
```

### macOS — Experiment B (`experiment.cpp`)

Builds and runs **Experiment B: Predator-Induced Colony Scatter and Satellite Group Formation** (Section VIII-B of the final report). This is the inverse of Experiment A (tight center spacing → supercolony merge and collapse): six colonies start at the dish **perimeter** (~9 units from center, 60° apart), beyond the 4.5-unit merge radius, so each colony forages independently. A **hungry amoeba** is placed near the east colony; **cocci are omitted** so predation concentrates on bacteria. Proximity fear is enabled so bacteria enter `ESCAPE` before contact, in addition to the post-hit disperse cascade.

```bash
./buildExperiment.sh
```

Expected emergent behavior over ~160 s of simulation time: repeated scatter-and-reorganize cycles, satellite colonies forming from detached pairs (group IDs ≥ 6), active group counts spiking (paper reports a peak of 21 groups and 18 satellites), and eventual consolidation into a smaller set of surviving colonies.

Output files (written to the project root):

- `experiment_scatter_log.csv` — time series every 0.5 s: active groups, satellite groups, detached count, dispersing groups, live bacteria, predator hits, pair formations
- `experiment_events.csv` — discrete events (experiment start, merges, dispersals, pair formations, predator hits)
- `experiment_screenshot_*.png` — optional screenshots (press **S** in-app)

Plot group dynamics (Fig. 8g in the report):

```bash
python graph.py   # reads experiment_scatter_log.csv → figB4_group_dynamics.png
```

Reference screenshots from a recorded run live in `assets/` (`b_4.2s.jpg`, `b_15.2s.jpg`, `b_57.4s.jpg`); sample CSVs are in `assets/experiment_results/`.

### Windows

```bash
./windowsBuild.sh
```

Requires raylib installed system-wide (`-lraylib`). Produces `main.exe`.

### Python setup (for plotting Experiment B)

```bash
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

---

## Controls

### Main demo (`main`)

| Key | Action |
|-----|--------|
| **Mouse** | Look around (free camera) |
| **WASD / arrows** | Move camera |
| **T** (hold) | Tight swarming — high cohesion |
| **X** (hold) | Dispersed — high separation |
| **R** (hold) | Reset boid weights to default |
| **G** | Toggle debug overlays |
| **F** | Toggle first-person view (follow a bacterium) |
| **N** | Cycle to next live bacterium (FPV mode) |
| **Esc / close window** | Quit |

Hover the crosshair over an organism to see its status HUD (hunger, behavior, temperature stress).

### Experiment B (`experiment`)

All main-demo keys above, plus:

| Key | Action |
|-----|--------|
| **Space** | Pause / resume simulation |
| **P** | Toggle CSV recording |
| **S** | Save screenshot |
| **R** | Reset boid parameters (press, not hold) |

Debug overlays and the experiment HUD (phase label, group counts, predator hits, pair formations) are on by default.

---

## Project layout

```
micro3d/
├── main.cpp              # Interactive demo (perimeter colonies; cocci enabled)
├── experiment.cpp        # Experiment B — predator scatter + satellite groups (Section VIII-B)
├── graph.py              # Plot Experiment B time series (Fig. 8g)
├── index.html            # Static project showcase / report companion page
├── project_proposal.md   # Early project proposal (superseded by final report)
├── requirements.txt      # Python deps for graph.py
│
├── buildnew.sh           # Build & run main (macOS)
├── buildExperiment.sh    # Build & run experiment (macOS)
├── windowsBuild.sh       # Build & run main (Windows)
│
├── src/                  # Simulation source (headers + one .cpp)
├── include/              # raylib headers + vendored Eigen
├── libraries/            # Prebuilt raylib dylib (macOS)
├── assets/               # Screenshots, videos, final report PDF, sample experiment CSVs
└── figures/              # Generated figure output (if present)
```

---

## Source files (`src/`)

Most simulation logic is header-only for easy inlining. Only `BoidBehavior.cpp` is compiled separately.

### Core physics

| File | Description |
|------|-------------|
| `Node.h` | Point mass with position, velocity, and force accumulation |
| `PhysicsBody.h` | Mass-spring soft body; implicit Euler integration with Eigen sparse solver |
| `ForceGenerator.h` | Abstract force interface applied each physics step |
| `GravityForce.h` | Constant gravitational acceleration |
| `BuoyancyForce.h` | Upward force based on fluid density and submersion |
| `DragForce.h` | Velocity-proportional hydrodynamic drag |
| `FluidEnvironment.h` | Shared fluid parameters (density, surface height, gravity) |

### Organisms

| File | Description |
|------|-------------|
| `Bacteria.h` | Flagellated rod bacterium — 4 body nodes + 14 flagellum nodes, motor control, rendering |
| `Amoeba.h` | Predatory amoeba — deformable membrane, organelles, hunting/seeking behaviors, custom shader |
| `Cocci.h` | Spherical cocci clusters (secondary prey/visual elements) |
| `Spirogyra.h` | Filamentous alga prototype (optional; commented out in main) |
| `Fish.h` | Early fish agent prototype |
| `FlagellumDrive.h` | Sinusoidal flagellar thrust helper |

### Behavior & ecology

| File | Description |
|------|-------------|
| `BehaviorStateMachine.h` | Internal states (hunger, fear, temperature stress), behavior selection, motor outputs |
| `BoidBehavior.h` / `.cpp` | Reynolds boids (separation, alignment, cohesion) as a `ForceGenerator` |
| `PopulationManager.h` | Proximity-based bacterial reproduction when energy needs are met |
| `Nutrient.h` | 3D nutrient particle field with Gaussian blobs; concentration sampling and visualization |
| `PetriDish.h` | Cylindrical arena, floor/walls, heat lamps, temperature field, isotherm rendering |
| `Obstacle.h` | Sphere and box collision obstacles |
| `ObstaclePerception.h` | Steering to avoid nearby obstacles |
| `Bait.h` | Simple attractor/bait helper |

### Rendering

| File | Description |
|------|-------------|
| `amoeba.vs` / `amoeba.fs` | GLSL shaders for the amoeba membrane (loaded at runtime from `src/`) |
| `Cube.h` | Debug/placeholder geometry |

### Entry points (project root)

| File | Description |
|------|-------------|
| `main.cpp` | General interactive demo — 6 perimeter-spaced bacterial colonies, amoeba predator, nutrients, obstacles, cocci clusters, live boid tuning (**T** / **X** / **R**). Commented block at ~line 154 holds the **Experiment A** tight-center spawn layout. |
| `experiment.cpp` | **Experiment B** (Section VIII-B) — perimeter colonies, hungry amoeba near east group, no cocci, proximity fear, experiment HUD, CSV logging. Window title: *"Micro-Life 3D — Experiment: Predator Scatter"*. |

---

## Experiments (final report, Section VIII)

The conference paper describes four experiments. Only **Experiment B** has a dedicated reproducible entry point in this repository.

| Experiment | Topic | How to reproduce |
|------------|-------|------------------|
| **A** | Colony spacing and merge dynamics — tight center placement → supercolony fusion and mass die-off | Swap `groupCenters` in `main.cpp` to the commented tight-center block (~lines 154–161); run `./buildnew.sh` |
| **B** | Predator-induced colony scatter and satellite group formation | `./buildExperiment.sh` → `experiment.cpp` |
| **C** | Temperature-seeking vs. foraging trade-off (chemotaxis vs. thermotaxis) | Run `main` and observe single-bacterium FPV/debug HUD in conflicting nutrient/temperature fields; video noted in report |
| **D** | Prey detection radius and amoeba survival (cocci-only food, scarce resources) | Separate parameter sweep; not bundled as a standalone binary |

### Experiment B setup (what `experiment.cpp` configures)

- **Colonies:** 6 groups × up to 32 bacteria (192 slots), 16 live per group at start, centers at radius ~9 on the dish floor
- **Predator:** Amoeba spawned at `(7.0, floorY + 2.5, 0.5)` with initial hunger boost (`feed(-70)`) near group 0 (east)
- **Prey:** Bacteria only — `cocciClusters` is empty
- **Fear:** Bacteria flee within predator sense radius before physical contact
- **Dynamics tracked:** Group merges, disperse timers after hits, detached rejoin (1.5 u) vs. pair formation (0.8 u), satellite groups (ID ≥ 6)
- **Metrics:** Logged every 0.5 s to `experiment_scatter_log.csv`; events to `experiment_events.csv`

---

## Assets

| Path | Contents |
|------|----------|
| `assets/report.pdf` | Final conference paper (*Micro-Life 3D: An Autonomous Ecosystem*) |
| `assets/*.png` | HUD screenshots, temperature visualization, behavior stills |
| `assets/video/` | Recorded demo clips (`amoeba-hunting.mp4`, `bacteria-behavior.mp4`, etc.) |
| `assets/experiment_results/` | Sample CSV output from a prior experiment run |

Open `index.html` in a browser to view the project page, screenshots, videos, and the linked final report at `assets/report.pdf`.

---

## How the simulation works (brief)

1. **Environment** — A `PetriDish` defines the arena, wall collisions, and a temperature field from external heat lamps. Nutrients concentrate along a target isotherm (~40 °C) where the amoeba prefers to hunt.

2. **Bacteria** — Each bacterium is a soft body steered by boid forces within its group. Groups spawn around the dish perimeter, merge when close, disperse after predator hits, and can reform via detached pair-formation. Internal hunger drives food seeking; fear drives escape from the amoeba.

3. **Amoeba** — A larger deformable predator with its own state machine: wanders when sated, hunts bacteria when hungry, and seeks warmer water when temperature-stressed.

4. **Physics loop** (each frame) — Snapshot boid states → update group metadata → apply merging/dispersal logic → integrate forces → resolve collisions → render.

For full system design (soft-body integration, behavioral hierarchy, amoeba locomotion, flocking tether, and experiment analysis), see the final report referenced from `index.html` and `project_proposal.md` for early scope notes.

---

## Troubleshooting

- **Shader not found** — Run the binary from the project root so `src/amoeba.vs` and `src/amoeba.fs` resolve correctly.
- **dylib not found (macOS)** — Keep `libraries/libraylib.5.5.0.dylib` next to the executable, or set `DYLD_LIBRARY_PATH=./libraries`.
- **SDK path error in build scripts** — Use the manual `clang++` command above with `SDK=$(xcrun --show-sdk-path)`.
- **Black window / no GL** — Ensure OpenGL-capable display; raylib needs a GPU context.

---
