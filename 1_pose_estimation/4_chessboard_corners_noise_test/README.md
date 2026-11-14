# Chessboard Corners Noise Test

This example evaluates how additive Gaussian noise affects OpenCV chessboard corner detection.

It:
- generates a synthetic chessboard image,
- adds Gaussian noise with configurable sigma,
- runs `findChessboardCorners` + `cornerSubPix` on both clean and noisy images,
- displays a side-by-side visualization and prints detection status.

## Build

```bash
cd 1_pose_estimation/4_chessboard_corners_noise_test
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/chessboard_corners_noise_test
```

With custom parameters:

```bash
./build/chessboard_corners_noise_test \
  --width=9 \
  --height=6 \
  --square_size_px=60 \
  --noise_sigma=30 \
  --seed=42
```

Help:

```bash
./build/chessboard_corners_noise_test --help
```

## Parameters

- `--width`: number of inner corners along board width (default: 9)
- `--height`: number of inner corners along board height (default: 6)
- `--square_size_px`: square size in pixels (default: 60)
- `--noise_sigma`: standard deviation of Gaussian noise (default: 20)
- `--seed`: RNG seed for repeatable noise (default: 42)

## Findings (measured on 2026-11-14)

### Method

- Binary built from this folder with OpenCV 4.10.0.
- Board setup: 9x6 inner corners, square size 60 px.
- For each sigma in the sweep, detection was tested for multiple seeds.
- In headless mode, runs used `QT_QPA_PLATFORM=offscreen` and `timeout 1s` to avoid waiting indefinitely on GUI input.

### Coarse sweep (5 seeds per sigma)

| Noise sigma | Noisy detection success | Clean detection success |
|---:|---:|---:|
| 0 | 5/5 | 5/5 |
| 10 | 5/5 | 5/5 |
| 20 | 5/5 | 5/5 |
| 30 | 5/5 | 5/5 |
| 40 | 5/5 | 5/5 |
| 50 | 5/5 | 5/5 |
| 60 | 1/5 | 5/5 |
| 70 | 0/5 | 5/5 |
| 80 | 0/5 | 5/5 |
| 90 | 0/5 | 5/5 |
| 100 | 0/5 | 5/5 |

### Fine sweep near transition (10 seeds per sigma)

| Noise sigma | Noisy detection success |
|---:|---:|
| 52 | 9/10 |
| 54 | 6/10 |
| 56 | 3/10 |
| 58 | 1/10 |
| 60 | 1/10 |
| 62 | 1/10 |
| 64 | 0/10 |
| 66 | 0/10 |
| 68 | 0/10 |

### Interpretation

- Detection is robust up to about sigma 50 for this synthetic setup.
- A steep degradation starts around sigma 54-58.
- Detection is effectively unusable from about sigma 64 and above.
- Clean-image detection remained stable in all tested runs.

These values are specific to this generated board size, contrast, and detection flags.
