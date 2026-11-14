#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace {

void generateRandomPlanarPoints(int pointCount, float xMax, float yMax,
                                uint64 seed, vector<Point3f> &points) {
  points.clear();
  points.reserve(pointCount);

  RNG rng(seed);
  for (int i = 0; i < pointCount; ++i) {
    points.push_back(
        Point3f(rng.uniform(0.0f, xMax), rng.uniform(0.0f, yMax), 0.0f));
  }
}

// getPerspectiveTransform and findHomography describe a pinhole camera, so
// the lens distortion has to be removed first. undistortPoints returns pixel
// coordinates when the camera matrix is passed as the new projection matrix P.
vector<Point2f> undistortPixels(const vector<Point2f> &points,
                                const Mat &cameraMatrix,
                                const Mat &distCoeffs) {
  vector<Point2f> undistorted;
  undistortPoints(
      points, undistorted, cameraMatrix, distCoeffs, noArray(), cameraMatrix,
      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 1e-12));
  return undistorted;
}

vector<Point2f> toPlanePoints(const vector<Point3f> &points, double scale) {
  vector<Point2f> planar;
  planar.reserve(points.size());
  for (const Point3f &p : points) {
    planar.push_back(Point2f(static_cast<float>(p.x * scale),
                             static_cast<float>(p.y * scale)));
  }
  return planar;
}

void addGaussianNoise(vector<Point2f> &points, double sigma, RNG &rng) {
  for (Point2f &p : points) {
    p.x += static_cast<float>(rng.gaussian(sigma));
    p.y += static_cast<float>(rng.gaussian(sigma));
  }
}

// Mean distance, in rectified pixels, between the transformed image points and
// their true positions on the rectified sheet.
double meanRectificationError(const Mat &transform,
                              const vector<Point2f> &imagePoints,
                              const vector<Point2f> &rectifiedPoints) {
  vector<Point2f> mapped;
  perspectiveTransform(imagePoints, mapped, transform);

  double total = 0.0;
  for (size_t i = 0; i < mapped.size(); ++i) {
    total += norm(mapped[i] - rectifiedPoints[i]);
  }
  return total / static_cast<double>(mapped.size());
}

// A homography is defined up to scale: normalize it so that H(2, 2) = 1.
Mat normalized(const Mat &H) { return H / H.at<double>(2, 2); }

double relativeDifference(const Mat &A, const Mat &B) {
  return norm(normalized(A) - normalized(B)) / norm(normalized(B));
}

// Test pattern for the sheet: a chessboard with a colored cell in each corner,
// so that the orientation of the rectified result is easy to check.
Mat generateSheetImage(const Size &size, int cellSizePx) {
  Mat sheet(size, CV_8UC3, Scalar(0, 0, 0));
  for (int y = 0; y * cellSizePx < size.height; ++y) {
    for (int x = 0; x * cellSizePx < size.width; ++x) {
      const uchar gray = (x + y) % 2 == 0 ? 235 : 90;
      rectangle(sheet,
                Rect(x * cellSizePx, y * cellSizePx, cellSizePx, cellSizePx),
                Scalar(gray, gray, gray), FILLED);
    }
  }

  const int lastX = (size.width / cellSizePx - 1) * cellSizePx;
  const int lastY = (size.height / cellSizePx - 1) * cellSizePx;
  rectangle(sheet, Rect(0, 0, cellSizePx, cellSizePx), Scalar(0, 0, 255),
            FILLED);
  rectangle(sheet, Rect(lastX, 0, cellSizePx, cellSizePx), Scalar(0, 200, 0),
            FILLED);
  rectangle(sheet, Rect(lastX, lastY, cellSizePx, cellSizePx),
            Scalar(255, 0, 0), FILLED);
  rectangle(sheet, Rect(0, lastY, cellSizePx, cellSizePx), Scalar(0, 220, 220),
            FILLED);
  return sheet;
}

Mat drawResult(const Size &imageSize, const Mat &cameraImage,
               const vector<Point2f> &corners, const Mat &rectified) {
  Mat view1 = cameraImage.clone();
  vector<Point> polygon;
  for (const Point2f &c : corners) {
    polygon.push_back(Point(cvRound(c.x), cvRound(c.y)));
  }
  polylines(view1, polygon, true, Scalar(0, 255, 255), 1, LINE_AA);
  for (const Point &p : polygon) {
    circle(view1, p, 4, Scalar(0, 255, 255), FILLED, LINE_AA);
  }
  putText(view1, "camera image, 4 image points", Point(10, 25),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1, LINE_AA);

  Mat view2(imageSize, CV_8UC3, Scalar(0, 0, 0));
  const Point offset((imageSize.width - rectified.cols) / 2,
                     (imageSize.height - rectified.rows) / 2);
  rectified.copyTo(view2(Rect(offset, rectified.size())));
  putText(view2, "getPerspectiveTransform + warpPerspective", Point(10, 25),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1, LINE_AA);

  Mat visualization;
  hconcat(view1, view2, visualization);
  return visualization;
}

void getPerspectiveTransformArtificialPoints(int interiorPointCount,
                                             double noiseSigmaPx, int trials,
                                             uint64 seed, bool noDisplay,
                                             const string &outputPath) {
  const Size imageSize(640, 480);

  // Planar sheet (like a sheet of paper) and the size of its rectified image.
  const float sheetWidth = 0.30f;
  const float sheetHeight = 0.20f;
  const double pixelsPerMeter = 1500.0;
  const Size rectifiedSize(cvRound(sheetWidth * pixelsPerMeter),
                           cvRound(sheetHeight * pixelsPerMeter));

  std::vector<double> cameraMatrixVec{535.915733961632,
                                      0,
                                      342.2831547330837,
                                      0,
                                      535.915733961632,
                                      235.5708290978817,
                                      0,
                                      0,
                                      1};
  const Mat1d cameraMatrix{3, 3, cameraMatrixVec.data()};

  std::vector<double> distCoeffsVec{-0.2663726090966068, -0.03858889892230465,
                                    0.001783194704285296,
                                    -0.0002812210044111547, 0.2383915308087849};
  const Mat1d distCoeffs{5, 1, distCoeffsVec.data()};

  // Synthetic camera pose (sheet frame to camera frame): the sheet is tilted,
  // so its image is a general quadrilateral.
  const Mat rvec = (Mat_<double>(3, 1) << -0.40, 0.30, 0.10);
  const Mat tvec = (Mat_<double>(3, 1) << -0.15, -0.08, 0.60);

  // The four sheet corners are the only points getPerspectiveTransform needs;
  // the interior points are used to evaluate the result.
  const vector<Point3f> sheetCorners{{0.0f, 0.0f, 0.0f},
                                     {sheetWidth, 0.0f, 0.0f},
                                     {sheetWidth, sheetHeight, 0.0f},
                                     {0.0f, sheetHeight, 0.0f}};
  vector<Point3f> interiorPoints;
  generateRandomPlanarPoints(interiorPointCount, sheetWidth, sheetHeight, seed,
                             interiorPoints);

  // Where these points must end up in the rectified image.
  const vector<Point2f> rectifiedCorners =
      toPlanePoints(sheetCorners, pixelsPerMeter);
  const vector<Point2f> rectifiedInterior =
      toPlanePoints(interiorPoints, pixelsPerMeter);
  vector<Point2f> rectifiedAll = rectifiedCorners;
  rectifiedAll.insert(rectifiedAll.end(), rectifiedInterior.begin(),
                      rectifiedInterior.end());

  // Pixels as the camera would see them: projectPoints applies the lens
  // distortion.
  vector<Point2f> distortedCorners, distortedInterior;
  projectPoints(sheetCorners, rvec, tvec, cameraMatrix, distCoeffs,
                distortedCorners);
  projectPoints(interiorPoints, rvec, tvec, cameraMatrix, distCoeffs,
                distortedInterior);

  // Noise-free pinhole pixels, used only to evaluate the transforms.
  const vector<Point2f> cleanCorners =
      undistortPixels(distortedCorners, cameraMatrix, distCoeffs);
  const vector<Point2f> cleanInterior =
      undistortPixels(distortedInterior, cameraMatrix, distCoeffs);

  // Ground truth transform from the image to the rectified sheet, computed
  // from the known pose: sheet point -> image is K * [r1 r2 t].
  Mat R;
  Rodrigues(rvec, R);
  const Mat poseColumns[] = {R.col(0), R.col(1), tvec};
  Mat sheetToImage;
  hconcat(poseColumns, 3, sheetToImage);
  sheetToImage = cameraMatrix * sheetToImage;
  const Mat sheetToRectified = (Mat_<double>(3, 3) << pixelsPerMeter, 0, 0, 0,
                                pixelsPerMeter, 0, 0, 0, 1);
  const Mat H_gt = normalized(sheetToRectified * sheetToImage.inv());

  cout << "Synthetic input points generated with projectPoints()." << endl;
  cout << "Sheet size: " << sheetWidth << " x " << sheetHeight << " m" << endl;
  cout << "Rectified image: " << rectifiedSize.width << "x"
       << rectifiedSize.height << " px (" << pixelsPerMeter << " px/m)" << endl;
  cout << "Random seed: " << seed << endl;
  cout << "Interior points count: " << interiorPoints.size() << endl;
  cout << "Pixel noise sigma: " << noiseSigmaPx << " px" << endl << endl;

  // 1. Noise-free corners: the four correspondences determine the homography
  // exactly, so it must match the ground truth.
  const Mat M_clean = getPerspectiveTransform(cleanCorners, rectifiedCorners);
  cout << "Ground truth transform image -> rectified (from the camera pose):\n"
       << H_gt << endl
       << endl;
  cout << "getPerspectiveTransform from noise-free corners:\n"
       << normalized(M_clean) << endl;
  cout << "  relative difference to ground truth: "
       << relativeDifference(M_clean, H_gt) << endl;
  cout << "  mean rectification error of the interior points: "
       << meanRectificationError(M_clean, cleanInterior, rectifiedInterior)
       << " px" << endl
       << endl;

  // 2. Noisy points. The first noise realization is printed and visualized,
  // all of them are averaged.
  RNG rng(seed + 1);
  double sumFourPoints = 0.0;
  double sumAllPoints = 0.0;
  Mat M_first;
  vector<Point2f> corners_first;
  for (int trial = 0; trial < trials; ++trial) {
    vector<Point2f> noisyCorners = distortedCorners;
    vector<Point2f> noisyInterior = distortedInterior;
    addGaussianNoise(noisyCorners, noiseSigmaPx, rng);
    addGaussianNoise(noisyInterior, noiseSigmaPx, rng);
    const vector<Point2f> corners =
        undistortPixels(noisyCorners, cameraMatrix, distCoeffs);
    vector<Point2f> allPoints = corners;
    const vector<Point2f> interior =
        undistortPixels(noisyInterior, cameraMatrix, distCoeffs);
    allPoints.insert(allPoints.end(), interior.begin(), interior.end());

    // Exactly four point pairs.
    const Mat M_four = getPerspectiveTransform(corners, rectifiedCorners);
    // Least squares fit to all the points.
    const Mat H_all = findHomography(allPoints, rectifiedAll);

    sumFourPoints +=
        meanRectificationError(M_four, cleanInterior, rectifiedInterior);
    sumAllPoints +=
        meanRectificationError(H_all, cleanInterior, rectifiedInterior);

    if (trial == 0) {
      M_first = M_four;
      corners_first = corners;

      const Mat H_four = findHomography(corners, rectifiedCorners);
      cout << "First noisy realization (" << noiseSigmaPx
           << " px noise on every image point):" << endl;
      cout << "getPerspectiveTransform from 4 noisy corners:\n"
           << normalized(M_four) << endl;
      cout << "  relative difference to findHomography on the same 4 points: "
           << relativeDifference(M_four, H_four) << endl;
      cout << "  mean rectification error of the interior points: "
           << meanRectificationError(M_four, cleanInterior, rectifiedInterior)
           << " px" << endl;
      cout << "findHomography on all " << allPoints.size()
           << " noisy points:" << endl;
      cout << "  mean rectification error of the interior points: "
           << meanRectificationError(H_all, cleanInterior, rectifiedInterior)
           << " px" << endl
           << endl;
    }
  }

  cout << "Average mean rectification error of the interior points over "
       << trials << " noise realizations:" << endl;
  cout << "  getPerspectiveTransform (4 corners): " << sumFourPoints / trials
       << " px" << endl;
  cout << "  findHomography (all " << rectifiedAll.size()
       << " points): " << sumAllPoints / trials << " px" << endl;

  // Visualization: render the sheet as the camera would see it (pinhole
  // model, so the transform is an exact homography) and rectify it with the
  // transform estimated from the first noisy corners.
  const Mat sheetImage = generateSheetImage(rectifiedSize, 50);
  const Mat sheetToImagePx = sheetToImage * sheetToRectified.inv();
  Mat cameraImage;
  warpPerspective(sheetImage, cameraImage, sheetToImagePx, imageSize,
                  INTER_LINEAR, BORDER_CONSTANT, Scalar(40, 40, 40));
  Mat rectified;
  warpPerspective(cameraImage, rectified, M_first, rectifiedSize, INTER_LINEAR,
                  BORDER_CONSTANT, Scalar(40, 40, 40));

  Mat visualization =
      drawResult(imageSize, cameraImage, corners_first, rectified);
  if (!outputPath.empty()) {
    imwrite(outputPath, visualization);
  }
  if (!noDisplay) {
    imshow("getPerspectiveTransform with artificial points", visualization);
    waitKey();
  }
}

const char *params =
    "{ help h         |       | print usage }"
    "{ points         | 54    | number of artificial interior points on the "
    "sheet }"
    "{ noise_sigma_px | 0.5   | standard deviation of Gaussian pixel noise }"
    "{ trials         | 200   | number of noise realizations used for the "
    "average errors }"
    "{ seed           | 42    | random seed used for point generation }"
    "{ no_display     | false | run without opening a GUI window }"
    "{ output         |       | optional path to save the visualization image "
    "}";

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example 4: perspective correction with getPerspectiveTransform on "
        "artificial points.\n");
    parser.printMessage();
    return 0;
  }
  if (!parser.check()) {
    parser.printErrors();
    return 1;
  }

  const int interiorPointCount = parser.get<int>("points");
  const double noiseSigmaPx = parser.get<double>("noise_sigma_px");
  const int trials = parser.get<int>("trials");
  const uint64 seed = static_cast<uint64>(parser.get<int>("seed"));
  const bool noDisplay = parser.get<bool>("no_display");
  const string outputPath = parser.get<string>("output");

  if (interiorPointCount < 1) {
    cerr << "points must be positive." << endl;
    return 1;
  }

  if (noiseSigmaPx < 0.0) {
    cerr << "noise_sigma_px must be non-negative." << endl;
    return 1;
  }

  if (trials < 1) {
    cerr << "trials must be positive." << endl;
    return 1;
  }

  getPerspectiveTransformArtificialPoints(interiorPointCount, noiseSigmaPx,
                                          trials, seed, noDisplay, outputPath);
  return 0;
}
