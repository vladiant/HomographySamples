# Circles Pattern Corners Displacement Test

This example evaluates how random circle-center displacement affects OpenCV circles-grid center detection.

It:
- generates a synthetic symmetric circles grid,
- randomly displaces each circle center within a configurable extent,
- runs `findCirclesGrid` on both ideal and displaced patterns,
- displays a side-by-side visualization and prints detection status.

## Build

```bash
cd 1_pose_estimation/6_circles_pattern_corners_displacement_test
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/circles_pattern_corners_displacement_test
```

With custom parameters:

```bash
./build/circles_pattern_corners_displacement_test \
  --width=9 \
  --height=6 \
  --cell_size_px=60 \
  --radius_px=14 \
  --displacement_px=8 \
  --seed=42
```

Headless (useful for repeated experiments):

```bash
./build/circles_pattern_corners_displacement_test --displacement_px=12 --seed=7 --no_display=true
```

Help:

```bash
./build/circles_pattern_corners_displacement_test --help
```

## Parameters

- `--width`: grid width in points (default: 9)
- `--height`: grid height in points (default: 6)
- `--cell_size_px`: center-to-center spacing in pixels (default: 60)
- `--radius_px`: circle radius in pixels (default: 14)
- `--displacement_px`: maximum absolute random center displacement in pixels (default: 5)
- `--seed`: RNG seed for repeatable displacement sampling (default: 42)
- `--no_display`: do not open visualization window (default: false)

## Findings (measured on 2026-11-14)

### Method

- Binary built from this folder with OpenCV 4.10.0.
- Grid setup: 9x6 points, cell size 60 px, radius 14 px.
- For each displacement value, multiple seeds were tested.
- `--no_display=true` was used to avoid GUI blocking during sweeps.

### Coarse sweep (5 seeds per displacement)

| Displacement extent (px) | Displaced detection success | Ideal detection success |
|---:|---:|---:|
| 0 | 5/5 | 5/5 |
| 2 | 5/5 | 5/5 |
| 4 | 5/5 | 5/5 |
| 6 | 5/5 | 5/5 |
| 8 | 5/5 | 5/5 |
| 10 | 5/5 | 5/5 |
| 12 | 2/5 | 5/5 |
| 14 | 0/5 | 5/5 |
| 16 | 0/5 | 5/5 |
| 18 | 0/5 | 5/5 |
| 20 | 0/5 | 5/5 |

### Fine sweep near transition (30 seeds per displacement)

| Displacement extent (px) | Displaced detection success |
|---:|---:|
| 9 | 29/30 |
| 10 | 24/30 |
| 11 | 17/30 |
| 12 | 12/30 |
| 13 | 7/30 |
| 14 | 0/30 |

### Interpretation

- Detection is very reliable up to about 9 px displacement in this setup.
- Reliability drops quickly from about 10 px to 13 px.
- Detection fails consistently at 14 px and above.
- Ideal-pattern detection stayed stable in all measured runs.

These thresholds are specific to this synthetic pattern geometry and detector settings.
