# Micro-Life 3D

A real-time 3D artificial-life simulation of microorganisms in a petri-dish aquatic environment. Flagellated bacteria swarm, feed on nutrient patches, and flee a predatory amoeba while responding to temperature gradients, obstacles, and internal hunger/fear states. Built in C++ with [raylib](https://www.raylib.com/) for rendering and visualization.

**Authors:** Jalal Ikram, Amanda An — CS 275 Final Project

---

## Features

- **Soft-body physics** — mass-spring bodies with flagellar propulsion and hydrodynamic drag/buoyancy
- **Autonomous agents** — behavior state machines drive wandering, food seeking, predator escape, temperature seeking, and obstacle avoidance
- **Boid swarming** — six bacterial colonies with separation, alignment, cohesion, group merging, dispersal, and pair formation
- **Ecosystem** — nutrient field tied to temperature isotherms, heat lamps, 3D obstacles, and cocci clusters
- **Experiment mode** — headless-friendly metrics logging to CSV for analysis and plotting
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

### macOS — experiment mode

Records group dynamics and predator-scatter events to CSV while the simulation runs:

```bash
./buildExperiment.sh
```

Output files (written to the project root):

- `experiment_scatter_log.csv` — time series (active groups, detached count, predator hits, etc.)
- `experiment_events.csv` — discrete events (merges, dispersals, pair formations)
- `experiment_screenshot_*.png` — optional screenshots (press **S** in-app)

### Windows

```bash
./windowsBuild.sh
```

Requires raylib installed system-wide (`-lraylib`). Produces `main.exe`.

### Plot experiment results

```bash
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python graph.py
```

`graph.py` reads `experiment_scatter_log.csv` and saves `figB4_group_dynamics.png`.

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

### Experiment mode (`experiment`)

All main-demo keys above, plus:

| Key | Action |
|-----|--------|
| **Space** | Pause / resume simulation |
| **P** | Toggle CSV recording |
| **S** | Save screenshot |
| **R** | Reset boid parameters (press, not hold) |

Debug overlays are on by default in experiment mode.

---

## Project layout

```
micro3d/
├── main.cpp              # Interactive ecosystem demo (primary entry point)
├── experiment.cpp        # Predator-scatter experiment with CSV logging
├── graph.py              # Matplotlib plot of experiment time series
├── index.html            # Static project showcase page
├── project_proposal.md   # Original project proposal and design doc
├── requirements.txt      # Python deps for graph.py
│
├── buildnew.sh           # Build & run main (macOS)
├── buildExperiment.sh    # Build & run experiment (macOS)
├── windowsBuild.sh       # Build & run main (Windows)
│
├── src/                  # Simulation source (headers + one .cpp)
├── include/              # raylib headers + vendored Eigen
├── libraries/            # Prebuilt raylib dylib (macOS)
├── assets/               # Screenshots, videos, sample experiment CSVs
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
| `main.cpp` | Full interactive simulation — 6 bacterial groups, amoeba predator, nutrients, obstacles, cocci, HUD |
| `experiment.cpp` | Same ecosystem configured for the "predator scatter" experiment; writes metrics CSVs |

---

## Assets

| Path | Contents |
|------|----------|
| `assets/*.png` | HUD screenshots, temperature visualization, behavior stills |
| `assets/video/` | Recorded demo clips (`amoeba-hunting.mp4`, `bacteria-behavior.mp4`, etc.) |
| `assets/experiment_results/` | Sample CSV output from a prior experiment run |

Open `index.html` in a browser to view the project page. Place `assets/report.pdf` there for the linked final report.

---

## How the simulation works (brief)

1. **Environment** — A `PetriDish` defines the arena, wall collisions, and a temperature field from external heat lamps. Nutrients concentrate along a target isotherm (~40 °C) where the amoeba prefers to hunt.

2. **Bacteria** — Each bacterium is a soft body steered by boid forces within its group. Groups spawn around the dish perimeter, merge when close, disperse after predator hits, and can reform via detached pair-formation. Internal hunger drives food seeking; fear drives escape from the amoeba.

3. **Amoeba** — A larger deformable predator with its own state machine: wanders when sated, hunts bacteria when hungry, and seeks warmer water when temperature-stressed.

4. **Physics loop** (each frame) — Snapshot boid states → update group metadata → apply merging/dispersal logic → integrate forces → resolve collisions → render.


---

## Troubleshooting

- **Shader not found** — Run the binary from the project root so `src/amoeba.vs` and `src/amoeba.fs` resolve correctly.
- **dylib not found (macOS)** — Keep `libraries/libraylib.5.5.0.dylib` next to the executable, or set `DYLD_LIBRARY_PATH=./libraries`.
- **SDK path error in build scripts** — Use the manual `clang++` command above with `SDK=$(xcrun --show-sdk-path)`.
- **Black window / no GL** — Ensure OpenGL-capable display; raylib needs a GPU context.

---
