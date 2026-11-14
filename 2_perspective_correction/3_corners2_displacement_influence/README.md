# Corners2 Displacement Influence on Homography Decomposition

This example is based on `2_perspective_correction/2_artificial_points_example` and investigates how random perturbations of `corners2` affect homography decomposition quality.

The pipeline:
- Generate synthetic planar `objectPoints`.
- Project points into two views to obtain ideal `corners1` and `corners2`.
- Randomly displace only `corners2` by a configurable pixel extent.
- Estimate homography with `findHomography(corners1, displacedCorners2)`.
- Decompose homography with `decomposeHomographyMat`.
- Compare decomposition to the known synthetic relative pose.

## Folder

`2_perspective_correction/3_corners2_displacement_influence`

## Build

```bash
cd 2_perspective_correction/3_corners2_displacement_influence
cmake -S . -B build
cmake --build build
```

## Run

Default sweep:

```bash
./build/corners2_displacement_influence
```

Example with explicit settings:

```bash
./build/corners2_displacement_influence \
  --max_disp=10 \
  --step=1 \
  --trials=50 \
  --seed=1000
```

Help:

```bash
./build/corners2_displacement_influence --help
```

## Parameters

- `--max_disp`: maximum random displacement magnitude (pixels)
- `--step`: displacement increment per sweep step (pixels)
- `--trials`: number of random trials per displacement step
- `--seed`: base seed for displacement randomization

## Output Meaning

For each displacement value, the program prints:
- `success_rate`: ratio of trials passing a baseline-relative quality criterion.
- `mean_best_rot_err_deg`: mean best rotation error across decomposition solutions.
- `mean_best_tdir_err_deg`: mean best translation-direction error across decomposition solutions.
- `mean_solutions`: average number of returned decomposition solutions.

Success criterion used in the run below:
- `best_rot_err <= baseline + 2 deg`
- `best_tdir_err <= baseline + 8 deg`

where baseline is the zero-displacement trial.

## Findings (measured on 2026-11-14)

Run command:

```bash
./build/corners2_displacement_influence --max_disp=10 --step=1 --trials=50 --seed=1000
```

Baseline:
- `baseline_best_rot_err_deg = 4.06389`
- `baseline_best_tdir_err_deg = 2.3112`

Sweep results:

| disp_px | success_rate | mean_best_rot_err_deg | mean_best_tdir_err_deg | mean_solutions |
|---:|---:|---:|---:|---:|
| 0 | 1.00 | 4.0639 | 2.3112 | 4.0 |
| 1 | 0.84 | 4.6107 | 3.7982 | 4.0 |
| 2 | 0.56 | 5.6542 | 6.3806 | 4.0 |
| 3 | 0.32 | 6.9227 | 9.0776 | 4.0 |
| 4 | 0.26 | 8.1925 | 11.8146 | 4.0 |
| 5 | 0.24 | 9.5444 | 14.4729 | 4.0 |
| 6 | 0.18 | 10.6588 | 16.8762 | 4.0 |
| 7 | 0.18 | 11.4200 | 19.0347 | 4.0 |
| 8 | 0.18 | 12.0576 | 20.9463 | 4.0 |
| 9 | 0.12 | 12.5388 | 22.7385 | 4.0 |
| 10 | 0.12 | 12.9100 | 24.5607 | 4.0 |

Interpretation:
- Increasing random displacement on `corners2` degrades decomposition quality monotonically.
- Rotation and translation-direction errors rise steadily with displacement.
- Relative-pass success falls from 1.00 at 0 px to about 0.12 at 9-10 px.
- `decomposeHomographyMat` still returns about 4 solutions consistently, but their agreement with ground-truth motion worsens.
