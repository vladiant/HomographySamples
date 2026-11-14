# Feature matching camera matrix from a video stream

This example follows the same OpenCV education style as the other homography samples in this repository.

The program opens a video stream, detects the chessboard in each frame, tracks corresponding points, and runs `calibrateCamera()` to estimate the intrinsic camera matrix `K`.

At the same time, ORB feature matching is used between consecutive frames to estimate a relative homography. The projection of the same planar pattern is used as a practical validation that the camera motion is consistent with the recovered geometry.

## Run

```bash
cmake -S . -B build
cmake --build build
./build/feature_matching_video_example --video path/to/video.mp4
```

## Arguments

- `--video`: path to the input video, or `0` for the default camera
- `--width`: chessboard width in corners
- `--height`: chessboard height in corners
- `--square_size`: square side length in meters
- `--max_frames`: maximum number of frames to process
