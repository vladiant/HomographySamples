#include <cmath>
#include <iostream>
#include <limits>
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

void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2,
                  const Mat &tvec2, Mat &R_1to2, Mat &tvec_1to2) {
  R_1to2 = R2 * R1.t();
  tvec_1to2 = R2 * (-R1.t() * tvec1) + tvec2;
}

double rotationErrorDeg(const Mat &R_est, const Mat &R_ref) {
  Mat dR = R_est * R_ref.t();
  const double tr = dR.at<double>(0, 0) + dR.at<double>(1, 1) + dR.at<double>(2, 2);
  const double cosAngle = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
  return acos(cosAngle) * 180.0 / CV_PI;
}

double directionErrorDegUpToSign(const Mat &v_est, const Mat &v_ref) {
  const double n_est = norm(v_est);
  const double n_ref = norm(v_ref);
  if (n_est <= 1e-12 || n_ref <= 1e-12) {
    return 180.0;
  }

  Mat ve = v_est / n_est;
  Mat vr = v_ref / n_ref;
  const double dotVal = ve.dot(vr);
  const double absDot = std::max(-1.0, std::min(1.0, std::abs(dotVal)));
  return acos(absDot) * 180.0 / CV_PI;
}

vector<Point2f> displaceCorners(const vector<Point2f> &corners, double maxDispPx,
                                uint64 seed) {
  vector<Point2f> displaced;
  displaced.reserve(corners.size());
  RNG rng(seed);

  for (const Point2f &p : corners) {
    const float dx = rng.uniform(static_cast<float>(-maxDispPx),
                                 static_cast<float>(maxDispPx));
    const float dy = rng.uniform(static_cast<float>(-maxDispPx),
                                 static_cast<float>(maxDispPx));
    displaced.push_back(Point2f(p.x + dx, p.y + dy));
  }

  return displaced;
}

struct TrialStats {
  double bestRotErrDeg = std::numeric_limits<double>::infinity();
  double bestTransDirErrDeg = std::numeric_limits<double>::infinity();
  int solutionCount = 0;
};

TrialStats evaluateDisplacementTrial(const vector<Point2f> &corners1,
                                     const vector<Point2f> &corners2,
                                     const Mat &cameraMatrix,
                                     const Mat &R_1to2_ref,
                                     const Mat &t_1to2_ref,
                                     double displacementPx, uint64 seed) {
  TrialStats stats;
  vector<Point2f> displacedCorners2 = displaceCorners(corners2, displacementPx, seed);

  Mat H = findHomography(corners1, displacedCorners2);
  if (H.empty()) {
    return stats;
  }

  vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
  stats.solutionCount =
      decomposeHomographyMat(H, cameraMatrix, Rs_decomp, ts_decomp, normals_decomp);

  for (int i = 0; i < stats.solutionCount; ++i) {
    const double rotErr = rotationErrorDeg(Rs_decomp[i], R_1to2_ref);
    const double transErr =
        directionErrorDegUpToSign(ts_decomp[i], t_1to2_ref);

    if (rotErr < stats.bestRotErrDeg) {
      stats.bestRotErrDeg = rotErr;
    }
    if (transErr < stats.bestTransDirErrDeg) {
      stats.bestTransDirErrDeg = transErr;
    }
  }

  return stats;
}

void runCorners2DisplacementStudy(double maxDispPx, double stepPx, int trialsPerStep,
                                  uint64 baseSeed) {
  const Size patternSize(9, 6);
  const float squareSize = 0.025f;
  const bool useRandomObjectPoints = true;
  const uint64 pointsSeed = 2026;

  vector<Point3f> objectPoints;
  if (useRandomObjectPoints) {
    generateRandomPlanarPoints(patternSize.width * patternSize.height,
                               (patternSize.width - 1) * squareSize,
                               (patternSize.height - 1) * squareSize,
                               pointsSeed, objectPoints);
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

  Mat R1_ref, R2_ref;
  Rodrigues(rvec1_est, R1_ref);
  Rodrigues(rvec2_est, R2_ref);

  Mat R_1to2_ref, t_1to2_ref;
  computeC2MC1(R1_ref, tvec1_est, R2_ref, tvec2_est, R_1to2_ref, t_1to2_ref);

  TrialStats baselineStats = evaluateDisplacementTrial(
      corners1, corners2, cameraMatrix, R_1to2_ref, t_1to2_ref, 0.0, baseSeed);
  const double baselineRotErr = baselineStats.bestRotErrDeg;
  const double baselineTdirErr = baselineStats.bestTransDirErrDeg;

  const double rotMarginDeg = 2.0;
  const double tdirMarginDeg = 8.0;

  cout << "Random corners2 displacement influence on homography decomposition"
       << endl;
  cout << "object_points_mode="
       << (useRandomObjectPoints ? "random planar" : "grid planar")
       << ", points_seed=" << pointsSeed << ", points=" << objectPoints.size()
       << ", trials_per_step=" << trialsPerStep
       << ", base_seed=" << baseSeed << endl;
  cout << "baseline_best_rot_err_deg=" << baselineRotErr
       << ", baseline_best_tdir_err_deg=" << baselineTdirErr << endl;
  cout << "success condition: best_rot_err <= baseline + " << rotMarginDeg
       << " deg and best_tdir_err <= baseline + " << tdirMarginDeg << " deg"
       << endl;
  cout << "Columns: disp_px, success_rate, mean_best_rot_err_deg, "
          "mean_best_tdir_err_deg, mean_solutions"
       << endl;

  for (double disp = 0.0; disp <= maxDispPx + 1e-9; disp += stepPx) {
    int successCount = 0;
    double sumRotErr = 0.0;
    double sumTdirErr = 0.0;
    double sumSolutions = 0.0;

    for (int trial = 0; trial < trialsPerStep; ++trial) {
      TrialStats stats = evaluateDisplacementTrial(
          corners1, corners2, cameraMatrix, R_1to2_ref, t_1to2_ref, disp,
          baseSeed + static_cast<uint64>(trial));

      if (stats.solutionCount > 0) {
        sumRotErr += stats.bestRotErrDeg;
        sumTdirErr += stats.bestTransDirErrDeg;
        sumSolutions += stats.solutionCount;

        const bool good =
            (stats.bestRotErrDeg <= baselineRotErr + rotMarginDeg &&
             stats.bestTransDirErrDeg <= baselineTdirErr + tdirMarginDeg);
        if (good) {
          successCount++;
        }
      }
    }

    const double successRate =
        static_cast<double>(successCount) / static_cast<double>(trialsPerStep);
    const double meanRotErr = sumRotErr / static_cast<double>(trialsPerStep);
    const double meanTdirErr = sumTdirErr / static_cast<double>(trialsPerStep);
    const double meanSolutions = sumSolutions / static_cast<double>(trialsPerStep);

    cout << "disp_px=" << disp << ", success_rate=" << successRate
         << ", mean_best_rot_err_deg=" << meanRotErr
         << ", mean_best_tdir_err_deg=" << meanTdirErr
         << ", mean_solutions=" << meanSolutions << endl;
  }
}

const char *params =
    "{ help h          |      | print usage }"
    "{ max_disp        | 10.0 | maximum corners2 displacement in pixels }"
    "{ step            | 1.0  | displacement step in pixels }"
    "{ trials          | 50   | number of random trials per displacement }"
    "{ seed            | 1000 | base seed for displacement sampling }";

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example: corners2 displacement influence on homography decomposition.\n");
    parser.printMessage();
    return 0;
  }

  const double maxDispPx = parser.get<double>("max_disp");
  const double stepPx = parser.get<double>("step");
  const int trialsPerStep = parser.get<int>("trials");
  const uint64 baseSeed = static_cast<uint64>(parser.get<int>("seed"));

  if (maxDispPx < 0.0 || stepPx <= 0.0 || trialsPerStep <= 0) {
    cerr << "Invalid arguments: require max_disp >= 0, step > 0, trials > 0."
         << endl;
    return 1;
  }

  runCorners2DisplacementStudy(maxDispPx, stepPx, trialsPerStep, baseSeed);
  return 0;
}
