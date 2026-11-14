# Corners2 Displacement Influence on Homography Decomposition

This example is based on `4_decompose_homography_matrix/1_artificial_points_example` and studies how random displacement of `corners2` affects homography decomposition.

Pipeline:
- Generate synthetic planar `objectPoints` (random planar by default).
- Project points into two synthetic views to obtain ideal `corners1` and `corners2`.
- Randomly displace only `corners2` by a configurable pixel extent.
- Estimate homography with `findHomography(corners1, displacedCorners2)`.
- Decompose with `decomposeHomographyMat`.
- Compare decomposition to the known relative camera motion.

## Folder

`4_decompose_homography_matrix/2_corners2_displacement_influence`

## Build

```bash
cd 4_decompose_homography_matrix/2_corners2_displacement_influence
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/corners2_displacement_influence
```

Run used for findings:

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
- `--step`: displacement increment (pixels)
- `--trials`: number of random trials per displacement value
- `--seed`: base random seed for displacements

## Reported Metrics

Per displacement value (`disp_px`), the program reports:
- `success_rate`: fraction of trials passing a baseline-relative criterion.
- `mean_best_rot_err_deg`: mean best rotation error among decomposition solutions.
- `mean_best_tdir_err_deg`: mean best translation-direction error (sign ambiguity handled).
- `mean_solutions`: mean number of decomposition solutions.

Success criterion:
- `best_rot_err <= baseline + 2 deg`
- `best_tdir_err <= baseline + 8 deg`

where baseline is the zero-displacement case.

## Findings (measured on 2026-11-14)

Run command:

```bash
./build/corners2_displacement_influence --max_disp=10 --step=1 --trials=50 --seed=1000
```

Baseline:
- `baseline_best_rot_err_deg = 3.99062`
- `baseline_best_tdir_err_deg = 2.18721`

Results:

| disp_px | success_rate | mean_best_rot_err_deg | mean_best_tdir_err_deg | mean_solutions |
|---:|---:|---:|---:|---:|
| 0 | 1.00 | 3.9906 | 2.1872 | 4.0 |
| 1 | 0.70 | 4.8078 | 3.9615 | 4.0 |
| 2 | 0.38 | 6.3834 | 6.9218 | 4.0 |
| 3 | 0.26 | 8.3628 | 10.1737 | 4.0 |
| 4 | 0.14 | 10.1777 | 13.5977 | 4.0 |
| 5 | 0.10 | 11.3532 | 16.8186 | 4.0 |
| 6 | 0.02 | 12.4002 | 19.7450 | 4.0 |
| 7 | 0.00 | 13.1185 | 21.5003 | 4.0 |
| 8 | 0.00 | 13.7193 | 23.2743 | 4.0 |
| 9 | 0.00 | 14.0929 | 24.4254 | 4.0 |
| 10 | 0.00 | 14.3934 | 25.4954 | 4.0 |

Interpretation:
- Increasing random displacement on `corners2` degrades homography decomposition quality.
- Rotation and translation-direction errors increase steadily with displacement.
- Success rate drops quickly and reaches zero by around `7 px` displacement in this setup.
- `decomposeHomographyMat` consistently returns around 4 solutions, but solution quality worsens.
