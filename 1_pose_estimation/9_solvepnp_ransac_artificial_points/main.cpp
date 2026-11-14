#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace {

// Uniformly distributed points inside a box centered at the object origin.
// A zero extent along an axis gives coplanar points (e.g. extent.z == 0
// generates a planar pattern in the z = 0 plane, like a chessboard).
void generateRandomPoints(int pointCount, const Point3f &extent, uint64 seed,
                          vector<Point3f> &points) {
  points.clear();
  points.reserve(pointCount);

  RNG rng(seed);
  auto sample = [&rng](float size) {
    return size > 0.0f ? rng.uniform(-0.5f * size, 0.5f * size) : 0.0f;
  };
  for (int i = 0; i < pointCount; ++i) {
    points.push_back(
        Point3f(sample(extent.x), sample(extent.y), sample(extent.z)));
  }
}

double rotationErrorDeg(const Mat &rvecA, const Mat &rvecB) {
  Mat RA, RB;
  Rodrigues(rvecA, RA);
  Rodrigues(rvecB, RB);
  const Mat R_err = RA * RB.t();
  const double cosAngle = std::clamp((trace(R_err)[0] - 1.0) / 2.0, -1.0, 1.0);
  return acos(cosAngle) * 180.0 / CV_PI;
}

// The median is used instead of the mean: a badly failed fit can place a few
// points almost on the camera plane, and their huge errors would dominate a
// mean while saying little about the fit as a whole.
double medianReprojectionError(const vector<Point3f> &objectPoints,
                               const Mat &rvec, const Mat &tvec,
                               const Mat &cameraMatrix, const Mat &distCoeffs,
                               const vector<Point2f> &referencePoints) {
  vector<Point2f> projected;
  projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projected);

  vector<double> errors;
  errors.reserve(projected.size());
  for (size_t i = 0; i < projected.size(); ++i) {
    errors.push_back(norm(projected[i] - referencePoints[i]));
  }
  nth_element(errors.begin(), errors.begin() + errors.size() / 2, errors.end());
  return errors[errors.size() / 2];
}

void printPoseResult(const string &name, const Mat &rvec, const Mat &tvec,
                     const Mat &rvecGt, const Mat &tvecGt,
                     const vector<Point3f> &objectPoints,
                     const vector<Point2f> &cleanImagePoints,
                     const Mat &cameraMatrix, const Mat &distCoeffs) {
  cout << name << ":" << endl;
  cout << "  rvec: " << rvec.t() << endl;
  cout << "  tvec: " << tvec.t() << endl;
  cout << "  rotation error: " << rotationErrorDeg(rvec, rvecGt) << " deg"
       << endl;
  cout << "  translation error: " << norm(tvec - tvecGt) << " m" << endl;
  cout << "  median reprojection error vs noise-free projections: "
       << medianReprojectionError(objectPoints, rvec, tvec, cameraMatrix,
                                  distCoeffs, cleanImagePoints)
       << " px" << endl
       << endl;
}

Mat drawResult(const Size &imageSize, const vector<Point2f> &imagePoints,
               const vector<char> &isRansacInlier, const Mat &cameraMatrix,
               const Mat &distCoeffs, const Mat &rvec, const Mat &tvec) {
  Mat img(imageSize, CV_8UC3, Scalar(0, 0, 0));

  int inliers = 0;
  for (size_t i = 0; i < imagePoints.size(); ++i) {
    if (isRansacInlier[i]) {
      circle(img, imagePoints[i], 3, Scalar(0, 200, 0), FILLED, LINE_AA);
      ++inliers;
    } else {
      drawMarker(img, imagePoints[i], Scalar(0, 0, 255), MARKER_TILTED_CROSS, 8,
                 1, LINE_AA);
    }
  }

  drawFrameAxes(img, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);

  putText(img,
          "solvePnPRansac: " + to_string(inliers) + "/" +
              to_string(imagePoints.size()) + " inliers",
          Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  putText(img, "green: inlier, red cross: rejected as outlier", Point(10, 50),
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
  return img;
}

void poseEstimationRansacArtificialPoints(int pointCount, double outlierRatio,
                                          double noiseSigmaPx,
                                          double reprojectionErrorPx,
                                          int iterations, bool planar,
                                          uint64 seed, bool noDisplay,
                                          const string &outputPath) {
  const Size imageSize(640, 480);

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

  // Artificial 3D points: a 3D cloud, or a coplanar set like a calibration
  // board when planar is requested.
  vector<Point3f> objectPoints;
  generateRandomPoints(pointCount, Point3f(0.30f, 0.20f, planar ? 0.0f : 0.20f),
                       seed, objectPoints);

  // Ground truth camera pose (object frame to camera frame).
  const Mat rvecGt = (Mat_<double>(3, 1) << 0.20, -0.35, 0.10);
  const Mat tvecGt = (Mat_<double>(3, 1) << 0.03, -0.02, 0.80);

  vector<Point2f> cleanImagePoints;
  projectPoints(objectPoints, rvecGt, tvecGt, cameraMatrix, distCoeffs,
                cleanImagePoints);

  // Observed points: Gaussian pixel noise on every point ...
  RNG rng(seed + 1);
  vector<Point2f> imagePoints = cleanImagePoints;
  for (Point2f &p : imagePoints) {
    p.x += static_cast<float>(rng.gaussian(noiseSigmaPx));
    p.y += static_cast<float>(rng.gaussian(noiseSigmaPx));
  }

  // ... and gross outliers: a random subset replaced by random pixels,
  // imitating wrong 2D-3D matches.
  vector<int> order(pointCount);
  iota(order.begin(), order.end(), 0);
  randShuffle(order, 1.0, &rng);
  const int outlierCount = cvRound(outlierRatio * pointCount);
  vector<char> isOutlier(pointCount, 0);
  for (int k = 0; k < outlierCount; ++k) {
    const int i = order[k];
    isOutlier[i] = 1;
    imagePoints[i] = Point2f(rng.uniform(0.0f, float(imageSize.width)),
                             rng.uniform(0.0f, float(imageSize.height)));
  }

  cout << "Synthetic input points generated with projectPoints()." << endl;
  cout << "Object points mode: " << (planar ? "random planar" : "random 3D")
       << endl;
  cout << "Random seed: " << seed << endl;
  cout << "Object points count: " << objectPoints.size() << endl;
  cout << "Pixel noise sigma: " << noiseSigmaPx << " px" << endl;
  cout << "Outliers: " << outlierCount << " of " << pointCount << endl;
  cout << "RANSAC reprojection error threshold: " << reprojectionErrorPx
       << " px" << endl
       << endl;

  cout << "Ground truth:" << endl;
  cout << "  rvec: " << rvecGt.t() << endl;
  cout << "  tvec: " << tvecGt.t() << endl << endl;

  // 1. Plain solvePnP uses every point, so the outliers corrupt the result.
  Mat rvecAll, tvecAll;
  solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecAll,
           tvecAll);
  printPoseResult("solvePnP (all points, outliers included)", rvecAll, tvecAll,
                  rvecGt, tvecGt, objectPoints, cleanImagePoints, cameraMatrix,
                  distCoeffs);

  // 2. solvePnPRansac finds the largest consensus set and refits on it.
  Mat rvecRansac, tvecRansac;
  vector<int> inlierIndices;
  const bool found = solvePnPRansac(
      objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecRansac,
      tvecRansac, false, iterations, static_cast<float>(reprojectionErrorPx),
      0.99, inlierIndices);
  if (!found) {
    cerr << "solvePnPRansac failed to find a pose." << endl;
    return;
  }
  printPoseResult("solvePnPRansac", rvecRansac, tvecRansac, rvecGt, tvecGt,
                  objectPoints, cleanImagePoints, cameraMatrix, distCoeffs);

  // 3. Reference: solvePnP on the points that are known to be good.
  vector<Point3f> goodObjectPoints;
  vector<Point2f> goodImagePoints;
  for (int i = 0; i < pointCount; ++i) {
    if (!isOutlier[i]) {
      goodObjectPoints.push_back(objectPoints[i]);
      goodImagePoints.push_back(imagePoints[i]);
    }
  }
  Mat rvecGood, tvecGood;
  solvePnP(goodObjectPoints, goodImagePoints, cameraMatrix, distCoeffs,
           rvecGood, tvecGood);
  printPoseResult("solvePnP (known good points only, reference)", rvecGood,
                  tvecGood, rvecGt, tvecGt, objectPoints, cleanImagePoints,
                  cameraMatrix, distCoeffs);

  // How well did RANSAC separate inliers from outliers?
  vector<char> isRansacInlier(pointCount, 0);
  for (int i : inlierIndices) {
    isRansacInlier[i] = 1;
  }
  int truePositives = 0;
  for (int i = 0; i < pointCount; ++i) {
    if (isRansacInlier[i] && !isOutlier[i]) {
      ++truePositives;
    }
  }
  const int goodCount = pointCount - outlierCount;
  cout << "RANSAC inliers: " << inlierIndices.size() << " of " << pointCount
       << " (" << goodCount << " points are not outliers)" << endl;
  cout << "  precision: "
       << (inlierIndices.empty()
               ? 0.0
               : double(truePositives) / double(inlierIndices.size()))
       << endl;
  cout << "  recall: "
       << (goodCount == 0 ? 0.0 : double(truePositives) / double(goodCount))
       << endl;

  Mat visualization =
      drawResult(imageSize, imagePoints, isRansacInlier, cameraMatrix,
                 distCoeffs, rvecRansac, tvecRansac);
  if (!outputPath.empty()) {
    imwrite(outputPath, visualization);
  }
  if (!noDisplay) {
    imshow("solvePnPRansac with artificial points", visualization);
    waitKey();
  }
}

const char *params =
    "{ help h             |       | print usage }"
    "{ points             | 100   | number of artificial 3D points }"
    "{ outlier_ratio      | 0.3   | fraction of image points replaced by "
    "random pixels }"
    "{ noise_sigma_px     | 0.5   | standard deviation of Gaussian pixel noise "
    "}"
    "{ reprojection_error | 3.0   | RANSAC inlier threshold in pixels }"
    "{ iterations         | 1000  | maximum number of RANSAC iterations }"
    "{ planar             | false | use coplanar points (z = 0) instead of a "
    "3D cloud }"
    "{ seed               | 42    | random seed used for point generation }"
    "{ no_display         | false | run without opening a GUI window }"
    "{ output             |       | optional path to save the visualization "
    "image }";

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example 7: robust pose estimation with solvePnPRansac on artificial "
        "points.\n");
    parser.printMessage();
    return 0;
  }
  if (!parser.check()) {
    parser.printErrors();
    return 1;
  }

  const int pointCount = parser.get<int>("points");
  const double outlierRatio = parser.get<double>("outlier_ratio");
  const double noiseSigmaPx = parser.get<double>("noise_sigma_px");
  const double reprojectionErrorPx = parser.get<double>("reprojection_error");
  const int iterations = parser.get<int>("iterations");
  const bool planar = parser.get<bool>("planar");
  const uint64 seed = static_cast<uint64>(parser.get<int>("seed"));
  const bool noDisplay = parser.get<bool>("no_display");
  const string outputPath = parser.get<string>("output");

  if (pointCount < 10) {
    cerr << "points must be at least 10." << endl;
    return 1;
  }

  if (outlierRatio < 0.0 || outlierRatio >= 1.0) {
    cerr << "outlier_ratio must be in the range [0, 1)." << endl;
    return 1;
  }

  if (noiseSigmaPx < 0.0) {
    cerr << "noise_sigma_px must be non-negative." << endl;
    return 1;
  }

  if (reprojectionErrorPx <= 0.0) {
    cerr << "reprojection_error must be positive." << endl;
    return 1;
  }

  if (iterations <= 0) {
    cerr << "iterations must be positive." << endl;
    return 1;
  }

  poseEstimationRansacArtificialPoints(pointCount, outlierRatio, noiseSigmaPx,
                                       reprojectionErrorPx, iterations, planar,
                                       seed, noDisplay, outputPath);
  return 0;
}
