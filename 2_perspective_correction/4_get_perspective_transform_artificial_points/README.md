# getPerspectiveTransform With Artificial Points

This example shows how to use `getPerspectiveTransform` for perspective correction with artificially generated points. It follows `2_perspective_correction/2_artificial_points_example`, but instead of estimating a homography from many correspondences it computes the transform from exactly four point pairs, the sheet corners, and checks it against the ground truth known from the camera pose.

What is generated:
- A planar sheet of 0.30 x 0.20 m, tilted in front of the synthetic camera, so its image is a general quadrilateral.
- The four sheet corners: the points that `getPerspectiveTransform` uses.
- `generateRandomPlanarPoints`: 54 random interior points on the sheet, used only to evaluate the result.
- The image points, generated with `projectPoints` (so they include lens distortion) and then undistorted with `undistortPoints`, because `getPerspectiveTransform` and `findHomography` describe a pinhole camera.

Then the code:
- computes the ground-truth transform from the image to the rectified sheet from the known pose, `K * [r1 r2 t]` inverted and scaled,
- shows that `getPerspectiveTransform` on noise-free corners matches it,
- adds Gaussian pixel noise and compares `getPerspectiveTransform` (4 corners only) with `findHomography` on the same four points and with `findHomography` on all points (least squares),
- averages the rectification error over many noise realizations,
- renders the sheet as the camera would see it and rectifies it with `warpPerspective`.

The rectification error is the mean distance, in pixels of the rectified image, between the transformed noise-free interior points and their true positions on the sheet.

## Folder

`2_perspective_correction/4_get_perspective_transform_artificial_points`

## Build

```bash
cd 2_perspective_correction/4_get_perspective_transform_artificial_points
cmake -S . -B build
cmake --build build
```

## Run

Default run:

```bash
./build/get_perspective_transform_artificial_points
```

With custom parameters:

```bash
./build/get_perspective_transform_artificial_points \
  --points=100 \
  --noise_sigma_px=1.0 \
  --trials=500 \
  --seed=7
```

Headless, saving the visualization to a file:

```bash
./build/get_perspective_transform_artificial_points --no_display=true --output=result.png
```

Help:

```bash
./build/get_perspective_transform_artificial_points --help
```

## Parameters

- `--points`: number of artificial interior points on the sheet (default: 54)
- `--noise_sigma_px`: standard deviation of the Gaussian pixel noise (default: 0.5)
- `--trials`: number of noise realizations used for the average errors (default: 200)
- `--seed`: RNG seed for repeatable points and noise (default: 42)
- `--no_display`: do not open the visualization window (default: false)
- `--output`: optional path to save the visualization image

## Tested Result (2026-11-15)

Built with OpenCV 4.6.0 (`-Wall -Wextra`, no warnings) and run with the default parameters: 54 interior points, 0.5 px noise, 200 trials, seed 42.

- Noise-free corners: `getPerspectiveTransform` matches the ground-truth transform with a relative difference of 3.3e-7 and a rectification error of 0.00003 px. The remaining difference comes from the single-precision (`float`) points that the function takes.
- `getPerspectiveTransform` and `findHomography` on the same four points give the same matrix (relative difference 2.8e-8), as expected for four exact correspondences.
- First noisy realization: rectification error 0.53 px for the 4 corners and 0.21 px for `findHomography` on all 58 points.
- Averaged over 200 noise realizations:

| Method | Mean rectification error |
|---|---:|
| `getPerspectiveTransform`, 4 corners | 0.78 px |
| `findHomography`, all 58 points | 0.23 px |

Interpretation:
- `getPerspectiveTransform` is exact for four correct correspondences, but with no redundancy every pixel of noise on a corner goes straight into the transform.
- With more correspondences than four, `findHomography` averages the noise out and was about 3.4 times more accurate here. Use `getPerspectiveTransform` when exactly four reliable points are available, such as the corners of a detected document, and `findHomography` otherwise.
