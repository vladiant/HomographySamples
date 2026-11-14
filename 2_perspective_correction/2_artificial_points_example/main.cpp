#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

namespace {

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

void generatePlanarGridPoints(Size boardSize, float squareSize,
                              vector<Point3f> &points,
                              Pattern patternType = CHESSBOARD) {
  points.resize(0);
  switch (patternType) {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          points.push_back(
              Point3f(float(j * squareSize), float(i * squareSize), 0));
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          points.push_back(Point3f(float((2 * j + i % 2) * squareSize),
                                   float(i * squareSize), 0));
      break;
    default:
      CV_Error(Error::StsBadArg, "Unknown pattern type\n");
  }
}

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

Mat computeHomography(const Mat &R_1to2, const Mat &tvec_1to2,
                      const double d_inv, const Mat &normal) {
  Mat homography = R_1to2 + d_inv * tvec_1to2 * normal.t();
  return homography;
}

void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2,
                  const Mat &tvec2, Mat &R_1to2, Mat &tvec_1to2) {
  // c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
  R_1to2 = R2 * R1.t();
  tvec_1to2 = R2 * (-R1.t() * tvec1) + tvec2;
}

void decomposeHomographySyntheticPoints() {
  const Size patternSize(9, 6);
  const float squareSize = 0.025f;
  const bool useRandomObjectPoints = true;
  const uint64 randomSeed = 42;

  vector<Point3f> objectPoints;
  if (useRandomObjectPoints) {
    const int pointCount = patternSize.width * patternSize.height;
    const float xMax = (patternSize.width - 1) * squareSize;
    const float yMax = (patternSize.height - 1) * squareSize;
    generateRandomPlanarPoints(pointCount, xMax, yMax, randomSeed,
                               objectPoints);
  } else {
    generatePlanarGridPoints(patternSize, squareSize, objectPoints);
  }

  std::vector<double> imgPoints{535.915733961632,
                                0,
                                342.2831547330837,
                                0,
                                535.915733961632,
                                235.5708290978817,
                                0,
                                0,
                                1};
  const Mat1d cameraMatrix{3, 3, imgPoints.data()};

  std::vector<double> distCoeffsVec{-0.2663726090966068, -0.03858889892230465,
                                    0.001783194704285296,
                                    -0.0002812210044111547, 0.2383915308087849};
  const Mat1d distCoeffs{5, 1, distCoeffsVec.data()};

  // Two synthetic camera poses used to project the same planar object points.
  const Mat rvec1 = (Mat_<double>(3, 1) << -0.08, 0.06, -0.03);
  const Mat tvec1 = (Mat_<double>(3, 1) << -0.07, -0.03, 0.65);
  const Mat rvec2 = (Mat_<double>(3, 1) << 0.06, -0.02, 0.08);
  const Mat tvec2 = (Mat_<double>(3, 1) << -0.02, 0.04, 0.72);

  vector<Point2f> corners1, corners2;
  projectPoints(objectPoints, rvec1, tvec1, cameraMatrix, distCoeffs, corners1);
  projectPoints(objectPoints, rvec2, tvec2, cameraMatrix, distCoeffs, corners2);

  Mat rvec1_est, tvec1_est;
  solvePnP(objectPoints, corners1, cameraMatrix, distCoeffs, rvec1_est,
           tvec1_est);

  Mat rvec2_est, tvec2_est;
  solvePnP(objectPoints, corners2, cameraMatrix, distCoeffs, rvec2_est,
           tvec2_est);

  Mat R1, R2;
  Rodrigues(rvec1_est, R1);
  Rodrigues(rvec2_est, R2);

  Mat R_1to2, t_1to2;
  computeC2MC1(R1, tvec1_est, R2, tvec2_est, R_1to2, t_1to2);

  Mat rvec_1to2;
  Rodrigues(R_1to2, rvec_1to2);

  Mat normal = (Mat_<double>(3, 1) << 0, 0, 1);
  Mat normal1 = R1 * normal;
  Mat origin(3, 1, CV_64F, Scalar(0));
  Mat origin1 = R1 * origin + tvec1_est;
  double d_inv1 = 1.0 / normal1.dot(origin1);

  Mat homography_euclidean = computeHomography(R_1to2, t_1to2, d_inv1, normal1);
  Mat homography = cameraMatrix * homography_euclidean * cameraMatrix.inv();
  homography /= homography.at<double>(2, 2);
  homography_euclidean /= homography_euclidean.at<double>(2, 2);

  vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
  int solutions = decomposeHomographyMat(homography, cameraMatrix, Rs_decomp,
                                         ts_decomp, normals_decomp);
  cout << "Synthetic input points generated with projectPoints()." << endl;
  cout << "Object points mode: "
       << (useRandomObjectPoints ? "random planar" : "grid planar") << endl;
  if (useRandomObjectPoints) {
    cout << "Random seed: " << randomSeed << endl;
  }
  cout << "Object points count: " << objectPoints.size() << endl;
  cout << "Decompose homography matrix computed from the camera displacement:"
       << endl
       << endl;

  for (int i = 0; i < solutions; i++) {
    double factor_d1 = 1.0 / d_inv1;
    Mat rvec_decomp;
    Rodrigues(Rs_decomp[i], rvec_decomp);
    cout << "Solution " << i << ":" << endl;
    cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
    cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
    cout << "tvec from homography decomposition: " << ts_decomp[i].t()
         << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
    cout << "tvec from camera displacement: " << t_1to2.t() << endl;
    cout << "plane normal from homography decomposition: "
         << normals_decomp[i].t() << endl;
    cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
  }

  Mat H = findHomography(corners1, corners2);
  solutions = decomposeHomographyMat(H, cameraMatrix, Rs_decomp, ts_decomp,
                                     normals_decomp);
  cout << "Decompose homography matrix estimated by findHomography():" << endl
       << endl;

  for (int i = 0; i < solutions; i++) {
    double factor_d1 = 1.0 / d_inv1;
    Mat rvec_decomp;
    Rodrigues(Rs_decomp[i], rvec_decomp);
    cout << "Solution " << i << ":" << endl;
    cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
    cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
    cout << "tvec from homography decomposition: " << ts_decomp[i].t()
         << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
    cout << "tvec from camera displacement: " << t_1to2.t() << endl;
    cout << "plane normal from homography decomposition: "
         << normals_decomp[i].t() << endl;
    cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  decomposeHomographySyntheticPoints();
  return 0;
}
