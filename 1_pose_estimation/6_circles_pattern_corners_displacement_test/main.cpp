#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace {

Mat generateDisplacedCirclesGridImage(const Size &patternSize, int cellSizePx,
                                      int radiusPx, double displacementPx,
                                      uint64 seed) {
  const int margin = cellSizePx * 2;
  const int width = margin * 2 + (patternSize.width - 1) * cellSizePx;
  const int height = margin * 2 + (patternSize.height - 1) * cellSizePx;

  Mat img(height, width, CV_8UC1, Scalar(0));
  RNG rng(seed);

  for (int y = 0; y < patternSize.height; ++y) {
    for (int x = 0; x < patternSize.width; ++x) {
      const double dx = rng.uniform(-displacementPx, displacementPx);
      const double dy = rng.uniform(-displacementPx, displacementPx);

      Point2d centerD(margin + x * cellSizePx + dx, margin + y * cellSizePx + dy);
      Point center(cvRound(centerD.x), cvRound(centerD.y));
      circle(img, center, radiusPx, Scalar(255), FILLED, LINE_AA);
    }
  }

  GaussianBlur(img, img, Size(3, 3), 0.8);
  return img;
}

Ptr<FeatureDetector> createCircleBlobDetector(int radiusPx) {
  SimpleBlobDetector::Params params;

  params.filterByColor = true;
  params.blobColor = 255;

  params.filterByArea = true;
  const float minArea = static_cast<float>(CV_PI * radiusPx * radiusPx * 0.2);
  const float maxArea = static_cast<float>(CV_PI * radiusPx * radiusPx * 3.0);
  params.minArea = minArea;
  params.maxArea = maxArea;

  params.filterByCircularity = true;
  params.minCircularity = 0.5f;

  params.filterByConvexity = true;
  params.minConvexity = 0.6f;

  params.filterByInertia = false;
  params.minDistBetweenBlobs = static_cast<float>(radiusPx);

  return SimpleBlobDetector::create(params);
}

bool detectCirclesGridCenters(const Mat &gray, const Size &patternSize,
                              int radiusPx, vector<Point2f> &centers) {
  auto blobDetector = createCircleBlobDetector(radiusPx);
  return findCirclesGrid(gray, patternSize, centers, CALIB_CB_SYMMETRIC_GRID,
                         blobDetector);
}

void testCirclesPatternDetectionWithDisplacement(const Size &patternSize,
                                                 int cellSizePx, int radiusPx,
                                                 double displacementPx,
                                                 uint64 seed, bool noDisplay) {
  Mat idealGray = generateDisplacedCirclesGridImage(patternSize, cellSizePx,
                                                     radiusPx, 0.0, seed);
  Mat displacedGray = generateDisplacedCirclesGridImage(
      patternSize, cellSizePx, radiusPx, displacementPx, seed);

  vector<Point2f> idealCenters;
  vector<Point2f> displacedCenters;

  bool idealFound =
      detectCirclesGridCenters(idealGray, patternSize, radiusPx, idealCenters);
  bool displacedFound = detectCirclesGridCenters(displacedGray, patternSize,
                                                 radiusPx, displacedCenters);

  Mat idealBgr, displacedBgr;
  cvtColor(idealGray, idealBgr, COLOR_GRAY2BGR);
  cvtColor(displacedGray, displacedBgr, COLOR_GRAY2BGR);

  drawChessboardCorners(idealBgr, patternSize, idealCenters, idealFound);
  drawChessboardCorners(displacedBgr, patternSize, displacedCenters,
                        displacedFound);

  putText(idealBgr,
          idealFound ? "ideal: centers detected" : "ideal: detection failed",
          Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7,
          idealFound ? Scalar(0, 200, 0) : Scalar(0, 0, 255), 2);

  putText(displacedBgr,
          displacedFound ? "displaced: centers detected"
                         : "displaced: detection failed",
          Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7,
          displacedFound ? Scalar(0, 200, 0) : Scalar(0, 0, 255), 2);

  Mat visualization;
  hconcat(idealBgr, displacedBgr, visualization);

  cout << "Pattern size: " << patternSize.width << "x" << patternSize.height
       << " centers" << endl;
  cout << "Cell size: " << cellSizePx << " px" << endl;
  cout << "Circle radius: " << radiusPx << " px" << endl;
  cout << "Displacement extent: " << displacementPx << " px" << endl;
  cout << "Seed: " << seed << endl;
  cout << "Ideal detection: " << (idealFound ? "success" : "failure") << endl;
  cout << "Displaced detection: " << (displacedFound ? "success" : "failure")
       << endl;

  if (!noDisplay) {
    imshow("Circles grid detection (ideal | displaced)", visualization);
    waitKey();
  }
}

const char *params =
    "{ help h          |      | print usage }"
    "{ width bw        | 9    | circles grid width (points) }"
    "{ height bh       | 6    | circles grid height (points) }"
    "{ cell_size_px    | 60   | center-to-center spacing in pixels }"
    "{ radius_px       | 14   | circle radius in pixels }"
    "{ displacement_px | 5.0  | max absolute random center displacement in pixels }"
    "{ seed            | 42   | random seed used for displacement generation }"
    "{ no_display      | false| run without opening a GUI window }";

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example 6: circles grid center detection with random displacement.\n");
    parser.printMessage();
    return 0;
  }

  const Size patternSize(parser.get<int>("width"), parser.get<int>("height"));
  const int cellSizePx = parser.get<int>("cell_size_px");
  const int radiusPx = parser.get<int>("radius_px");
  const double displacementPx = parser.get<double>("displacement_px");
  const uint64 seed = static_cast<uint64>(parser.get<int>("seed"));
  const bool noDisplay = parser.get<bool>("no_display");

  if (patternSize.width <= 1 || patternSize.height <= 1) {
    cerr << "Pattern dimensions must be greater than 1." << endl;
    return 1;
  }

  if (cellSizePx <= 0) {
    cerr << "cell_size_px must be positive." << endl;
    return 1;
  }

  if (radiusPx <= 0 || radiusPx >= cellSizePx / 2) {
    cerr << "radius_px must be positive and less than half of cell_size_px."
         << endl;
    return 1;
  }

  if (displacementPx < 0.0) {
    cerr << "displacement_px must be non-negative." << endl;
    return 1;
  }

  testCirclesPatternDetectionWithDisplacement(patternSize, cellSizePx, radiusPx,
                                              displacementPx, seed, noDisplay);
  return 0;
}
