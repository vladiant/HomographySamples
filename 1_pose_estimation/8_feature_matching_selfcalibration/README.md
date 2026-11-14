# Camera matrix self-calibration from video features (no chessboard)

This example follows the same OpenCV education style as the other homography samples in this repository.

Unlike `7_feature_matching_video_example`, this program does **not** require a chessboard or any known calibration target. It estimates the intrinsic camera matrix `K` directly from the natural features of an arbitrary scene captured in a video stream.

## How it works

1. ORB features are detected and matched between pairs of frames with enough baseline.
2. A fundamental matrix `F` is estimated robustly with RANSAC from the matched points.
3. The focal length is recovered in closed form from `F` using the Bougnoux formula, assuming the principal point is at the image center and square pixels.
4. Many frame pairs are processed and the per-pair focal estimates are combined with a median for robustness against outliers.
5. The resulting camera matrix is validated by estimating the essential matrix and recovering the relative pose.

The approach is based on classic auto-calibration: when the scene is general (not planar) and the camera moves, the epipolar geometry constrains the unknown focal length.

## Run

```bash
cmake -S . -B build
cmake --build build
./build/feature_matching_selfcalibration --video path/to/video.mp4
```

## Arguments

- `--video`: path to the input video, or `0` for the default camera
- `--max_frames`: maximum number of frames to process
- `--stride`: frame spacing between the two views of a pair (larger gives more baseline)
- `--min_inliers`: minimum number of RANSAC inliers required to accept a pair

## Notes

- The scene must be **non-planar** and the camera must **translate** (not only rotate). A pure rotation or a flat scene makes the focal length unobservable from two views.
- The principal point is assumed to be at the image center, which is a common and reasonable approximation.
