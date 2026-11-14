# Artificial Points Homography From Camera Displacement

This example follows the structure of `3_homography_camera_displacement/0_initial_example`, but uses artificially generated correspondences instead of detected corners from images.

What is generated:
- `objectPoints`: synthetic planar points.
- `corners1`, `corners2`: 2D image points generated from `projectPoints` using two synthetic camera poses.

Additional changes:
- `calcChessboardCorners` was renamed to `generatePlanarGridPoints`.
- A new helper `generateRandomPlanarPoints` was added to generate uniformly distributed planar points.
- The example uses random planar points by default.

## Folder

`3_homography_camera_displacement/1_artificial_points_example`

## Build

```bash
cd /home/vladiant/Workplace/HomographySamples/3_homography_camera_displacement/1_artificial_points_example
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/artificial_points_example
```

## What It Prints

The program prints:
- estimated homography from `findHomography(corners1, corners2)`,
- homography from camera displacement,
- homography from absolute camera poses,
- mean reprojection error of each homography against synthetic `corners2`.

## Tested Result (2026-11-14)

Build and run succeeded.

Observed output summary:
- object points mode: `random planar`
- random seed: `1337`
- object points count: `54`
- mean reprojection errors (pixels):
  - `findHomography` vs corners2: `0.0666887`
  - from displacement vs corners2: `1.18535`
  - from absolute poses vs corners2: `1.18535`

Interpretation:
- The synthetic pipeline runs end-to-end correctly.
- Homographies from displacement and absolute poses match each other closely in this setup.
- `findHomography` fits synthetic correspondences more tightly (lower reprojection error), as expected for direct fit to image points.
