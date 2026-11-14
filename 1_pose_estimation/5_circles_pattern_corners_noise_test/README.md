# Circles Pattern Corners Noise Test

This example evaluates how additive Gaussian noise affects OpenCV circles-grid center detection.

It:
- generates a synthetic symmetric circles grid,
- adds Gaussian noise with configurable sigma,
- runs `findCirclesGrid` on both clean and noisy images,
- displays a side-by-side visualization and prints detection status.

## Build

```bash
cd 1_pose_estimation/5_circles_pattern_corners_noise_test
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/circles_pattern_corners_noise_test
```

With custom parameters:

```bash
./build/circles_pattern_corners_noise_test \
  --width=9 \
  --height=6 \
  --cell_size_px=60 \
  --radius_px=14 \
  --noise_sigma=30 \
  --seed=42
```

Headless (useful for automated tests):

```bash
./build/circles_pattern_corners_noise_test --noise_sigma=40 --seed=7 --no_display=true
```

Help:

```bash
./build/circles_pattern_corners_noise_test --help
```

## Parameters

- `--width`: grid width in points (default: 9)
- `--height`: grid height in points (default: 6)
- `--cell_size_px`: center-to-center spacing in pixels (default: 60)
- `--radius_px`: circle radius in pixels (default: 14)
- `--noise_sigma`: standard deviation of Gaussian noise (default: 20)
- `--seed`: RNG seed for repeatable noise (default: 42)
- `--no_display`: do not open visualization window (default: false)

## Findings (measured on 2026-11-14)

### Method

- Binary built from this folder with OpenCV 4.10.0.
- Grid setup: 9x6 points, cell size 60 px, radius 14 px.
- For each sigma in the sweep, detection was tested for multiple seeds.
- `--no_display=true` was used to avoid GUI blocking during repeated runs.

### Coarse sweep (5 seeds per sigma)

| Noise sigma | Noisy detection success | Clean detection success |
|---:|---:|---:|
| 0 | 5/5 | 5/5 |
| 10 | 5/5 | 5/5 |
| 20 | 5/5 | 5/5 |
| 30 | 5/5 | 5/5 |
| 40 | 5/5 | 5/5 |
| 50 | 5/5 | 5/5 |
| 60 | 2/5 | 5/5 |
| 70 | 0/5 | 5/5 |
| 80 | 0/5 | 5/5 |
| 90 | 0/5 | 5/5 |
| 100 | 0/5 | 5/5 |

### Fine sweep near transition (10 seeds per sigma)

| Noise sigma | Noisy detection success |
|---:|---:|
| 52 | 10/10 |
| 54 | 9/10 |
| 56 | 8/10 |
| 58 | 7/10 |
| 60 | 4/10 |
| 62 | 3/10 |
| 64 | 1/10 |
| 66 | 0/10 |
| 68 | 0/10 |

### Interpretation

- Detection is consistently robust through about sigma 50.
- Reliability declines progressively from about sigma 54 to 64.
- Detection is effectively unusable from about sigma 66 and above.
- Clean-image detection stayed stable in all measured runs.

These thresholds are specific to this synthetic grid geometry, contrast, and detector settings.
