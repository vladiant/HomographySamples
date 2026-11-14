# Corners2 Displacement Influence on Homography Decomposition

This example is based on `3_homography_camera_displacement/1_artificial_points_example` and investigates how random perturbations of `corners2` affect homography decomposition quality.

Pipeline:
- Generate synthetic planar `objectPoints` (random planar by default).
- Project points with two synthetic camera poses to get ideal `corners1` and `corners2`.
- Apply random per-point displacement only to `corners2`.
- Estimate homography with `findHomography(corners1, displacedCorners2)`.
- Decompose with `decomposeHomographyMat` and compare with known relative camera motion.

## Folder

`3_homography_camera_displacement/2_corners2_displacement_influence`

## Build

```bash
cd /home/vladiant/Workplace/HomographySamples/3_homography_camera_displacement/2_corners2_displacement_influence
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/corners2_displacement_influence
```

Example run used for findings:

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

- `--max_disp`: maximum random displacement magnitude in pixels
- `--step`: displacement step size in pixels
- `--trials`: number of random trials per displacement step
- `--seed`: base random seed for displacement generation

## Metrics

For each displacement value, the program prints:
- `success_rate`: fraction of trials that satisfy a baseline-relative criterion.
- `mean_best_rot_err_deg`: mean best rotation error (deg) over decomposition solutions.
- `mean_best_tdir_err_deg`: mean best translation-direction error (deg, sign ambiguity handled).
- `mean_solutions`: mean number of decomposition solutions.

Success criterion:
- `best_rot_err <= baseline + 2 deg`
- `best_tdir_err <= baseline + 8 deg`

where baseline comes from the zero-displacement case.

## Findings (measured on 2026-11-14)

Run command:

```bash
./build/corners2_displacement_influence --max_disp=10 --step=1 --trials=50 --seed=1000
```

Baseline:
- `baseline_best_rot_err_deg = 3.80409`
- `baseline_best_tdir_err_deg = 2.03702`

Results:

| disp_px | success_rate | mean_best_rot_err_deg | mean_best_tdir_err_deg | mean_solutions |
|---:|---:|---:|---:|---:|
| 0 | 1.00 | 3.8041 | 2.0370 | 4.0 |
| 1 | 0.88 | 4.0008 | 4.1856 | 4.0 |
| 2 | 0.46 | 5.2020 | 7.6778 | 4.0 |
| 3 | 0.26 | 6.9464 | 11.5797 | 4.0 |
| 4 | 0.16 | 8.8169 | 15.6056 | 4.0 |
| 5 | 0.12 | 10.4963 | 19.4108 | 4.0 |
| 6 | 0.10 | 11.7680 | 22.6016 | 4.0 |
| 7 | 0.08 | 12.7435 | 25.1686 | 4.0 |
| 8 | 0.08 | 13.2550 | 27.4805 | 4.0 |
| 9 | 0.04 | 13.6890 | 29.2293 | 4.0 |
| 10 | 0.04 | 13.8310 | 30.4312 | 4.0 |

Interpretation:
- Increasing random displacement on `corners2` causes monotonic degradation in decomposition quality.
- Success rate drops quickly (1.00 at 0 px to 0.46 at 2 px, and 0.04 by 9-10 px).
- Both rotation and translation-direction errors increase steadily with displacement.
- `decomposeHomographyMat` consistently returns around 4 solutions, but their quality relative to the true motion degrades.
