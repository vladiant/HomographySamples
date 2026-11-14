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

void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2,
                  const Mat &tvec2, Mat &R_1to2, Mat &tvec_1to2) {
  // c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
  R_1to2 = R2 * R1.t();
  tvec_1to2 = R2 * (-R1.t() * tvec1) + tvec2;
}

Mat skewSymmetric(const Mat &v) {
  return (Mat_<double>(3, 3) << 0, -v.at<double>(2), v.at<double>(1),
          v.at<double>(2), 0, -v.at<double>(0), -v.at<double>(1),
          v.at<double>(0), 0);
}

// Fundamental matrix of two views with known intrinsics and relative pose:
// x2^T * F * x1 = 0 with F = K^-T * [t]x * R * K^-1 (E = [t]x * R).
Mat computeFundamentalMatrix(const Mat &cameraMatrix, const Mat &R_1to2,
                             const Mat &tvec_1to2) {
  const Mat essential = skewSymmetric(tvec_1to2) * R_1to2;
  const Mat K_inv = cameraMatrix.inv();
  return K_inv.t() * essential * K_inv;
}

// The fundamental matrix is defined up to scale and sign: scale it to unit
// Frobenius norm and pick the sign that agrees best with the reference.
Mat normalizeLike(const Mat &F, const Mat &reference) {
  Mat result = F / norm(F);
  if (result.dot(reference / norm(reference)) < 0.0) {
    result = -result;
  }
  return result;
}

// The fundamental matrix describes pinhole cameras, so the lens distortion
// has to be removed first. undistortPoints returns pixel coordinates when the
// camera matrix is passed as the new projection matrix P.
vector<Point2f> undistortPixels(const vector<Point2f> &points,
                                const Mat &cameraMatrix,
                                const Mat &distCoeffs) {
  vector<Point2f> undistorted;
  undistortPoints(
      points, undistorted, cameraMatrix, distCoeffs, noArray(), cameraMatrix,
      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 1e-12));
  return undistorted;
}

// Mean symmetric distance of corresponding points to their epipolar lines.
double meanEpipolarDistance(const Mat &F, const vector<Point2f> &points1,
                            const vector<Point2f> &points2) {
  double total = 0.0;
  for (size_t i = 0; i < points1.size(); ++i) {
    const Mat x1 = (Mat_<double>(3, 1) << points1[i].x, points1[i].y, 1.0);
    const Mat x2 = (Mat_<double>(3, 1) << points2[i].x, points2[i].y, 1.0);
    const Mat line2 = F * x1;      // epipolar line of x1 in the second image
    const Mat line1 = F.t() * x2;  // epipolar line of x2 in the first image
    const double d2 =
        abs(line2.dot(x2)) / hypot(line2.at<double>(0), line2.at<double>(1));
    const double d1 =
        abs(line1.dot(x1)) / hypot(line1.at<double>(0), line1.at<double>(1));
    total += 0.5 * (d1 + d2);
  }
  return total / static_cast<double>(points1.size());
}

void printFundamentalResult(const string &name, const Mat &F, const Mat &F_gt,
                            const vector<Point2f> &cleanPoints1,
                            const vector<Point2f> &cleanPoints2,
                            const vector<Point2f> &heldOutPoints1,
                            const vector<Point2f> &heldOutPoints2) {
  cout << name << ":" << endl;
  if (F.empty() || F.rows != 3 || F.cols != 3) {
    cout << "  no fundamental matrix was found" << endl << endl;
    return;
  }

  const Mat F_normalized = normalizeLike(F, F_gt);
  cout << "  F (unit Frobenius norm):\n" << F_normalized << endl;
  cout << "  det(F): " << determinant(F_normalized) << endl;
  cout << "  distance to ground-truth F: " << norm(F_normalized - F_gt) << endl;
  cout << "  mean epipolar distance, noise-free sample points: "
       << meanEpipolarDistance(F_normalized, cleanPoints1, cleanPoints2)
       << " px" << endl;
  cout << "  mean epipolar distance, held-out off-plane points: "
       << meanEpipolarDistance(F_normalized, heldOutPoints1, heldOutPoints2)
       << " px" << endl
       << endl;
}

Scalar colorFromIndex(int index, int count) {
  Mat hsv(1, 1, CV_8UC3,
          Scalar(static_cast<uchar>(180 * index / max(count, 1)), 255, 255));
  Mat bgr;
  cvtColor(hsv, bgr, COLOR_HSV2BGR);
  return Scalar(bgr.at<Vec3b>(0, 0)[0], bgr.at<Vec3b>(0, 0)[1],
                bgr.at<Vec3b>(0, 0)[2]);
}

void drawEpipolarLine(Mat &img, const Vec3f &epiline, const Scalar &color) {
  const double a = epiline[0], b = epiline[1], c = epiline[2];
  const double w = img.cols, h = img.rows;
  Point2d p1, p2;
  if (abs(b) > abs(a)) {
    p1 = Point2d(0.0, -c / b);
    p2 = Point2d(w, -(c + a * w) / b);
  } else {
    p1 = Point2d(-c / a, 0.0);
    p2 = Point2d(-(c + b * h) / a, h);
  }
  line(img, Point(cvRound(p1.x), cvRound(p1.y)),
       Point(cvRound(p2.x), cvRound(p2.y)), color, 1, LINE_AA);
}

Mat drawEpipolarGeometry(const Size &imageSize, const Mat &F,
                         const vector<Point2f> &points1,
                         const vector<Point2f> &points2,
                         const vector<uchar> &inlierMask) {
  Mat view1(imageSize, CV_8UC3, Scalar(0, 0, 0));
  Mat view2(imageSize, CV_8UC3, Scalar(0, 0, 0));

  vector<Point2f> selected1, selected2;
  const size_t maxLines = 12;
  for (size_t i = 0; i < points1.size(); ++i) {
    if (inlierMask[i]) {
      if (selected1.size() < maxLines) {
        selected1.push_back(points1[i]);
        selected2.push_back(points2[i]);
      }
    } else {
      drawMarker(view1, points1[i], Scalar(0, 0, 160), MARKER_TILTED_CROSS, 6,
                 1, LINE_AA);
      drawMarker(view2, points2[i], Scalar(0, 0, 160), MARKER_TILTED_CROSS, 6,
                 1, LINE_AA);
    }
  }

  vector<Vec3f> lines1, lines2;
  computeCorrespondEpilines(selected2, 2, F, lines1);
  computeCorrespondEpilines(selected1, 1, F, lines2);

  for (size_t i = 0; i < selected1.size(); ++i) {
    const Scalar color =
        colorFromIndex(static_cast<int>(i), static_cast<int>(maxLines));
    drawEpipolarLine(view1, lines1[i], color);
    drawEpipolarLine(view2, lines2[i], color);
    circle(view1, selected1[i], 4, color, FILLED, LINE_AA);
    circle(view2, selected2[i], 4, color, FILLED, LINE_AA);
  }

  putText(view1, "view 1: epipolar lines of view 2 points", Point(10, 25),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1, LINE_AA);
  putText(view2, "view 2: epipolar lines of view 1 points", Point(10, 25),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1, LINE_AA);
  putText(view1, "red cross: match rejected by RANSAC", Point(10, 50),
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1, LINE_AA);

  Mat visualization;
  hconcat(view1, view2, visualization);
  return visualization;
}

void fundamentalMatrixArtificialPoints(int pointCount, double outlierRatio,
                                       double noiseSigmaPx,
                                       double ransacThresholdPx, bool planar,
                                       uint64 seed, bool noDisplay,
                                       const string &outputPath) {
  const Size imageSize(640, 480);
  const int heldOutCount = 300;

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

  // Two synthetic camera poses (object frame to camera frame). The cameras sit
  // about 0.3 m apart, to the left and to the right of the object, and both
  // look at its center.
  const Mat rvec1 = (Mat_<double>(3, 1) << 0.10, -0.19, 0.04);
  const Mat tvec1 = (Mat_<double>(3, 1) << -0.001, -0.098, 0.808);
  const Mat rvec2 = (Mat_<double>(3, 1) << 0.06, 0.19, -0.04);
  const Mat tvec2 = (Mat_<double>(3, 1) << 0.013, -0.039, 0.862);

  // Artificial 3D points: a 3D cloud, or a coplanar set when planar is
  // requested. Coplanar points are a degenerate configuration for the
  // fundamental matrix.
  vector<Point3f> objectPoints;
  generateRandomPoints(pointCount, Point3f(0.30f, 0.20f, planar ? 0.0f : 0.20f),
                       seed, objectPoints);

  // Held-out points that are never used for the estimation. They are always a
  // 3D cloud, so they reveal whether the estimated F is correct for the whole
  // scene and not only for the points it was fitted to.
  vector<Point3f> heldOutObjectPoints;
  generateRandomPoints(heldOutCount, Point3f(0.30f, 0.20f, 0.20f), seed + 1000,
                       heldOutObjectPoints);

  // Ground truth fundamental matrix from the known poses and intrinsics.
  Mat R1, R2;
  Rodrigues(rvec1, R1);
  Rodrigues(rvec2, R2);
  Mat R_1to2, t_1to2;
  computeC2MC1(R1, tvec1, R2, tvec2, R_1to2, t_1to2);
  Mat F_gt = computeFundamentalMatrix(cameraMatrix, R_1to2, t_1to2);
  F_gt /= norm(F_gt);

  // Pixels as the cameras would see them: projectPoints applies the lens
  // distortion.
  vector<Point2f> distorted1, distorted2;
  projectPoints(objectPoints, rvec1, tvec1, cameraMatrix, distCoeffs,
                distorted1);
  projectPoints(objectPoints, rvec2, tvec2, cameraMatrix, distCoeffs,
                distorted2);

  // Noise-free pinhole pixels, used only to evaluate the estimated matrices.
  const vector<Point2f> clean1 =
      undistortPixels(distorted1, cameraMatrix, distCoeffs);
  const vector<Point2f> clean2 =
      undistortPixels(distorted2, cameraMatrix, distCoeffs);

  vector<Point2f> heldOutDistorted1, heldOutDistorted2;
  projectPoints(heldOutObjectPoints, rvec1, tvec1, cameraMatrix, distCoeffs,
                heldOutDistorted1);
  projectPoints(heldOutObjectPoints, rvec2, tvec2, cameraMatrix, distCoeffs,
                heldOutDistorted2);
  const vector<Point2f> heldOut1 =
      undistortPixels(heldOutDistorted1, cameraMatrix, distCoeffs);
  const vector<Point2f> heldOut2 =
      undistortPixels(heldOutDistorted2, cameraMatrix, distCoeffs);

  // Observed matches: Gaussian pixel noise in both images ...
  RNG rng(seed + 1);
  for (size_t i = 0; i < distorted1.size(); ++i) {
    distorted1[i].x += static_cast<float>(rng.gaussian(noiseSigmaPx));
    distorted1[i].y += static_cast<float>(rng.gaussian(noiseSigmaPx));
    distorted2[i].x += static_cast<float>(rng.gaussian(noiseSigmaPx));
    distorted2[i].y += static_cast<float>(rng.gaussian(noiseSigmaPx));
  }

  // ... and gross mismatches: a random subset of the points in the second
  // image is replaced by random pixels.
  vector<int> order(pointCount);
  iota(order.begin(), order.end(), 0);
  randShuffle(order, 1.0, &rng);
  const int outlierCount = cvRound(outlierRatio * pointCount);
  vector<char> isOutlier(pointCount, 0);
  for (int k = 0; k < outlierCount; ++k) {
    const int i = order[k];
    isOutlier[i] = 1;
    distorted2[i] = Point2f(rng.uniform(0.0f, float(imageSize.width)),
                            rng.uniform(0.0f, float(imageSize.height)));
  }

  // findFundamentalMat works on undistorted pixel coordinates.
  const vector<Point2f> points1 =
      undistortPixels(distorted1, cameraMatrix, distCoeffs);
  const vector<Point2f> points2 =
      undistortPixels(distorted2, cameraMatrix, distCoeffs);

  cout << "Synthetic input points generated with projectPoints()." << endl;
  cout << "Object points mode: " << (planar ? "random planar" : "random 3D")
       << endl;
  cout << "Random seed: " << seed << endl;
  cout << "Object points count: " << objectPoints.size() << endl;
  cout << "Pixel noise sigma: " << noiseSigmaPx << " px" << endl;
  cout << "Mismatches: " << outlierCount << " of " << pointCount << endl;
  cout << "RANSAC epipolar distance threshold: " << ransacThresholdPx << " px"
       << endl;
  cout << "Held-out off-plane validation points: " << heldOutCount << endl
       << endl;

  // The ground truth is evaluated like the estimates: its epipolar distances
  // are numerically zero for noise-free points.
  printFundamentalResult("Ground truth fundamental matrix", F_gt, F_gt, clean1,
                         clean2, heldOut1, heldOut2);

  // 1. The 8-point algorithm uses every match, so mismatches ruin the result.
  const Mat F_all = findFundamentalMat(points1, points2, FM_8POINT);
  printFundamentalResult("findFundamentalMat FM_8POINT (all matches)", F_all,
                         F_gt, clean1, clean2, heldOut1, heldOut2);

  // 2. RANSAC finds the largest set of matches consistent with one F.
  vector<uchar> inlierMask;
  const Mat F_ransac = findFundamentalMat(
      points1, points2, FM_RANSAC, ransacThresholdPx, 0.99, 2000, inlierMask);
  printFundamentalResult("findFundamentalMat FM_RANSAC", F_ransac, F_gt, clean1,
                         clean2, heldOut1, heldOut2);

  // 3. Reference: the 8-point algorithm on the matches that are known to be
  // good.
  vector<Point2f> goodPoints1, goodPoints2;
  for (int i = 0; i < pointCount; ++i) {
    if (!isOutlier[i]) {
      goodPoints1.push_back(points1[i]);
      goodPoints2.push_back(points2[i]);
    }
  }
  const Mat F_good = findFundamentalMat(goodPoints1, goodPoints2, FM_8POINT);
  printFundamentalResult(
      "findFundamentalMat FM_8POINT (known good matches only, reference)",
      F_good, F_gt, clean1, clean2, heldOut1, heldOut2);

  if (F_ransac.empty() || inlierMask.size() != points1.size()) {
    cerr << "FM_RANSAC failed to find a fundamental matrix." << endl;
    return;
  }

  // How well did RANSAC separate good matches from mismatches?
  int inliers = 0, truePositives = 0;
  for (int i = 0; i < pointCount; ++i) {
    if (inlierMask[i]) {
      ++inliers;
      if (!isOutlier[i]) {
        ++truePositives;
      }
    }
  }
  const int goodCount = pointCount - outlierCount;
  cout << "RANSAC inliers: " << inliers << " of " << pointCount << " ("
       << goodCount << " matches are not mismatches)" << endl;
  cout << "  precision: "
       << (inliers == 0 ? 0.0 : double(truePositives) / double(inliers))
       << endl;
  cout << "  recall: "
       << (goodCount == 0 ? 0.0 : double(truePositives) / double(goodCount))
       << endl;

  Mat visualization =
      drawEpipolarGeometry(imageSize, F_ransac, points1, points2, inlierMask);
  if (!outputPath.empty()) {
    imwrite(outputPath, visualization);
  }
  if (!noDisplay) {
    imshow("findFundamentalMat with artificial points", visualization);
    waitKey();
  }
}

const char *params =
    "{ help h               |       | print usage }"
    "{ points               | 100   | number of artificial 3D points }"
    "{ outlier_ratio        | 0.2   | fraction of matches replaced by random "
    "pixels in view 2 }"
    "{ noise_sigma_px       | 0.5   | standard deviation of Gaussian pixel "
    "noise }"
    "{ ransac_threshold_px  | 2.0   | RANSAC inlier threshold: distance to the "
    "epipolar line in pixels }"
    "{ planar               | false | use coplanar points (z = 0), a "
    "degenerate case for F }"
    "{ seed                 | 42    | random seed used for point generation }"
    "{ no_display           | false | run without opening a GUI window }"
    "{ output               |       | optional path to save the visualization "
    "image }";

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example 7: fundamental matrix estimation with findFundamentalMat on "
        "artificial points.\n");
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
  const double ransacThresholdPx = parser.get<double>("ransac_threshold_px");
  const bool planar = parser.get<bool>("planar");
  const uint64 seed = static_cast<uint64>(parser.get<int>("seed"));
  const bool noDisplay = parser.get<bool>("no_display");
  const string outputPath = parser.get<string>("output");

  if (pointCount < 16) {
    cerr << "points must be at least 16." << endl;
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

  if (ransacThresholdPx <= 0.0) {
    cerr << "ransac_threshold_px must be positive." << endl;
    return 1;
  }

  fundamentalMatrixArtificialPoints(pointCount, outlierRatio, noiseSigmaPx,
                                    ransacThresholdPx, planar, seed, noDisplay,
                                    outputPath);
  return 0;
}
