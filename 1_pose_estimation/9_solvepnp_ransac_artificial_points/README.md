# solvePnPRansac With Artificial Points

This example shows robust camera pose estimation with `solvePnPRansac` on artificially generated correspondences. It follows the pose estimation examples in `1_pose_estimation`, but instead of measured corners it uses synthetic 2D-3D matches, so the true pose is known and the result can be checked exactly.

What is generated:
- `objectPoints`: random 3D points in a box (default), or coplanar points in the `z = 0` plane with `--planar=true`, like a calibration board.
- `imagePoints`: `projectPoints` of those points for a known ground-truth pose, with Gaussian pixel noise added to every point.
- Outliers: a random subset of the image points (default 30%) is replaced by random pixels, imitating wrong 2D-3D matches.

Then the code estimates the pose three ways and compares each with the ground truth:
- `solvePnP` on all points, outliers included.
- `solvePnPRansac`, which finds the largest consensus set and refits on it.
- `solvePnP` on only the points that are known to be good, as a reference for the best achievable result with this noise.

For each pose it prints the rotation error (degrees), the translation error (meters) and the median reprojection error against the noise-free projections. The median is used because a badly failed fit can put a few points almost on the camera plane, and their huge errors would dominate a mean. It also prints how well RANSAC separated inliers from outliers (precision and recall), and shows the points with the estimated pose axes.

## Folder

`1_pose_estimation/9_solvepnp_ransac_artificial_points`

## Build

```bash
cd 1_pose_estimation/9_solvepnp_ransac_artificial_points
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/solvepnp_ransac_artificial_points
```

With custom parameters:

```bash
./build/solvepnp_ransac_artificial_points \
  --points=150 \
  --outlier_ratio=0.4 \
  --noise_sigma_px=1.0 \
  --reprojection_error=4 \
  --planar=true \
  --seed=7
```

Headless, saving the visualization to a file:

```bash
./build/solvepnp_ransac_artificial_points --no_display=true --output=result.png
```

Help:

```bash
./build/solvepnp_ransac_artificial_points --help
```

## Parameters

- `--points`: number of artificial 3D points (default: 100)
- `--outlier_ratio`: fraction of image points replaced by random pixels, in `[0, 1)` (default: 0.3)
- `--noise_sigma_px`: standard deviation of the Gaussian pixel noise (default: 0.5)
- `--reprojection_error`: RANSAC inlier threshold in pixels (default: 3.0)
- `--iterations`: maximum number of RANSAC iterations (default: 1000)
- `--planar`: use coplanar points (`z = 0`) instead of a 3D cloud (default: false)
- `--seed`: RNG seed for repeatable points, noise and outliers (default: 42)
- `--no_display`: do not open the visualization window (default: false)
- `--output`: optional path to save the visualization image

## Tested Result (2026-11-15)

Built with OpenCV 4.6.0 (`-Wall -Wextra`, no warnings) and run with the default parameters: 100 points, 30 outliers, 0.5 px noise, seed 42.

| Method | Rotation error | Translation error | Median reprojection error |
|---|---:|---:|---:|
| `solvePnP`, all points | 177.2 deg | 0.849 m | 44019 px |
| `solvePnPRansac` | 0.118 deg | 0.92 mm | 0.109 px |
| `solvePnP`, good points only (reference) | 0.118 deg | 0.92 mm | 0.109 px |

- RANSAC kept 70 of 100 points with precision 1 and recall 1, so it found exactly the 70 points that are not outliers and gave the same pose as the reference.
- With `--planar=true` the result is similar: `solvePnP` on all points is off by 172.6 deg, while `solvePnPRansac` gives 0.60 deg and 0.16 mm with the same 70 inliers. The rotation error is larger than for the 3D cloud (0.12 deg) in this run, as coplanar points constrain the pose less well.

Interpretation:
- Plain `solvePnP` was not merely less accurate with outliers, it failed completely. In all 12 combinations tested (outlier ratios 10%, 20%, 30% and 50%, each with three seeds) its rotation error was above 10 deg, while `solvePnPRansac` stayed within 2 deg every time. In the default run the failed fit even ended with a negative depth, hence the absurd reprojection error.
- The exact failure values of the non-robust fit can differ between OpenCV versions; the robust result is the stable one.
- As a general rule, the RANSAC threshold (`--reprojection_error`) should be a few times the pixel noise: too small rejects good points, too large accepts outliers.
