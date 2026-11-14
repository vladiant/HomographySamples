# Artificial Points Fundamental Matrix Example

This example shows how to estimate the fundamental matrix with `findFundamentalMat` from artificially generated correspondences. It is the general two-view counterpart of the homography examples in this repository: a homography maps points between two views of a *plane*, while the fundamental matrix relates two views of an arbitrary scene through the epipolar constraint `x2^T * F * x1 = 0`.

What is generated:
- `objectPoints`: random 3D points in a box (default), or coplanar points in the `z = 0` plane with `--planar=true`.
- `distorted1`, `distorted2`: 2D points generated with `projectPoints` from two synthetic camera poses about 0.3 m apart, so they include lens distortion like a real camera.
- Noise and mismatches: Gaussian pixel noise on all points, and a random subset (default 20%) of the points in the second image replaced by random pixels.
- Held-out points: 300 extra points, always a 3D cloud, that are never used for the estimation.

Then the code:
- removes the lens distortion with `undistortPoints` (the fundamental matrix describes pinhole cameras),
- computes the ground-truth `F = K^-T * [t]x * R * K^-1` from the known poses,
- estimates `F` with `FM_8POINT` on all matches, with `FM_RANSAC`, and with `FM_8POINT` on only the good matches as a reference,
- evaluates every matrix with the distance to the ground truth and with the mean epipolar distance, both on the noise-free sample points and on the held-out points,
- draws epipolar lines of the RANSAC result in both views.

The held-out points matter because of a degenerate case: for coplanar points many different matrices fit the correspondences equally well, so a small error on the sample points does not prove that `F` is correct for the scene.

## Folder

`4_decompose_homography_matrix/3_fundamental_matrix_artificial_points`

## Build

```bash
cd 4_decompose_homography_matrix/3_fundamental_matrix_artificial_points
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/artificial_points_example
```

Planar degenerate case, without noise and mismatches:

```bash
./build/artificial_points_example --planar=true --noise_sigma_px=0 --outlier_ratio=0
```

Headless, saving the visualization to a file:

```bash
./build/artificial_points_example --no_display=true --output=result.png
```

Help:

```bash
./build/artificial_points_example --help
```

## Parameters

- `--points`: number of artificial 3D points (default: 100)
- `--outlier_ratio`: fraction of matches replaced by random pixels in view 2, in `[0, 1)` (default: 0.2)
- `--noise_sigma_px`: standard deviation of the Gaussian pixel noise (default: 0.5)
- `--ransac_threshold_px`: RANSAC inlier threshold, the distance to the epipolar line in pixels (default: 2.0)
- `--planar`: use coplanar points (`z = 0`), a degenerate case for `F` (default: false)
- `--seed`: RNG seed for repeatable points, noise and mismatches (default: 42)
- `--no_display`: do not open the visualization window (default: false)
- `--output`: optional path to save the visualization image

## Tested Result (2026-11-15)

Built with OpenCV 4.6.0 (`-Wall -Wextra`, no warnings) and run with the default parameters: 100 points, 20 mismatches, 0.5 px noise, RANSAC threshold 2 px, seed 42.

Mean epipolar distance in pixels (smaller is better):

| Matrix | Distance to ground-truth F | Sample points | Held-out points |
|---|---:|---:|---:|
| Ground truth | 0 | 0.000007 | 0.000008 |
| `FM_8POINT`, all matches | 0.0113 | 40.7 | 41.6 |
| `FM_RANSAC` | 0.00070 | 0.27 | 0.30 |
| `FM_8POINT`, good matches only (reference) | 0.00028 | 0.16 | 0.17 |

- RANSAC kept 76 of 100 matches with precision 1 and recall 0.95, and came close to the reference that is built from the known-good matches.
- The ground-truth row evaluates to numerically zero, which confirms the synthetic data and the ground-truth matrix are consistent.

Planar points (`--planar=true`), where `F` is degenerate:

| Run | Matrix | Sample points | Held-out points |
|---|---|---:|---:|
| noise-free, no mismatches | `FM_8POINT` | 0.000008 | 0.33 |
| noise-free, no mismatches | `FM_RANSAC` | 0.000018 | 7.83 |
| default noise and mismatches | `FM_RANSAC` | 0.46 | 7.47 |

Interpretation:
- The 8-point algorithm alone is ruined by mismatches (about 41 px), while RANSAC removes them.
- For coplanar points the estimated `F` fits the sample points essentially perfectly even without any noise, yet is wrong for points off the plane: 7.8 px for `FM_RANSAC` on noise-free data, against 0.0005 px for the 3D cloud under the same conditions. With the default noise and mismatches it is 7.5 px, against 0.30 px for the 3D cloud. The correspondences simply do not determine `F` in that case. Use a homography for planar scenes, and a 3D scene with points at different depths for `findFundamentalMat`.
- Which of the many consistent matrices is returned is essentially arbitrary (the two methods in the table give 0.33 px and 7.8 px), so only the contrast with the 3D case is meaningful, not the exact planar numbers.
