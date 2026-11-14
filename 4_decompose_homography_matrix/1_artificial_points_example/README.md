# Artificial Points Decompose Homography Matrix Example

This example follows `4_decompose_homography_matrix/0_initial_example`, but uses artificially generated correspondences instead of detected corners from real images.

What is generated:
- `objectPoints`: synthetic planar points.
- `corners1`, `corners2`: 2D image points generated with `projectPoints` from two synthetic camera poses.

Additional changes:
- `calcChessboardCorners` is renamed to `generatePlanarGridPoints`.
- A new `generateRandomPlanarPoints` helper generates uniformly distributed planar points.
- Random planar points are used by default.

## Folder

`4_decompose_homography_matrix/1_artificial_points_example`

## Build

```bash
cd /home/vladiant/Workplace/HomographySamples/4_decompose_homography_matrix/1_artificial_points_example
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/artificial_points_example
```

## What It Does

- Solves pose for both synthetic views using `solvePnP`.
- Computes relative camera displacement.
- Builds a homography from displacement and decomposes it with `decomposeHomographyMat`.
- Builds a homography from `findHomography(corners1, corners2)` and decomposes it for comparison.

## Tested Result (2026-11-14)

Configure, build, and run completed successfully.

Observed output summary:
- object points mode: `random planar`
- random seed: `2026`
- object points count: `54`
- decomposition from camera-displacement homography: `4` solutions
- decomposition from `findHomography` homography: `4` solutions

This confirms the synthetic pipeline works end-to-end and reproduces expected multi-solution homography decomposition behavior.
