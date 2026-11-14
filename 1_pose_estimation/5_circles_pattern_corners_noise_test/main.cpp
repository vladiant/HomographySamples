#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace {

Mat generateCirclesGridImage(const Size &patternSize, int cellSizePx,
                             int radiusPx) {
  const int margin = cellSizePx;
  const int width = margin * 2 + (patternSize.width - 1) * cellSizePx;
  const int height = margin * 2 + (patternSize.height - 1) * cellSizePx;

  Mat img(height, width, CV_8UC1, Scalar(0));
  for (int y = 0; y < patternSize.height; ++y) {
    for (int x = 0; x < patternSize.width; ++x) {
      Point center(margin + x * cellSizePx, margin + y * cellSizePx);
      circle(img, center, radiusPx, Scalar(255), FILLED, LINE_AA);
    }
  }

  GaussianBlur(img, img, Size(3, 3), 0.8);
  return img;
}

Mat addGaussianNoise(const Mat &gray, double sigma, uint64 seed) {
  CV_Assert(gray.type() == CV_8UC1);

  Mat noise(gray.size(), CV_16SC1);
  RNG rng(seed);
  rng.fill(noise, RNG::NORMAL, 0.0, sigma);

  Mat gray16;
  gray.convertTo(gray16, CV_16SC1);

  Mat noisy16 = gray16 + noise;
  Mat noisy;
  noisy16.convertTo(noisy, CV_8UC1);
  return noisy;
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

bool detectCirclesGridCorners(const Mat &gray, const Size &patternSize,
                              int radiusPx, vector<Point2f> &corners) {
  auto blobDetector = createCircleBlobDetector(radiusPx);
  return findCirclesGrid(gray, patternSize, corners, CALIB_CB_SYMMETRIC_GRID,
                         blobDetector);
}

void testCirclesPatternDetectionWithNoise(const Size &patternSize, int cellSizePx,
                                          int radiusPx, double noiseSigma,
                                          uint64 seed, bool noDisplay) {
  Mat cleanGray = generateCirclesGridImage(patternSize, cellSizePx, radiusPx);
  Mat noisyGray = addGaussianNoise(cleanGray, noiseSigma, seed);

  vector<Point2f> cleanCenters;
  vector<Point2f> noisyCenters;

  bool cleanFound =
      detectCirclesGridCorners(cleanGray, patternSize, radiusPx, cleanCenters);
  bool noisyFound =
      detectCirclesGridCorners(noisyGray, patternSize, radiusPx, noisyCenters);

  Mat cleanBgr, noisyBgr;
  cvtColor(cleanGray, cleanBgr, COLOR_GRAY2BGR);
  cvtColor(noisyGray, noisyBgr, COLOR_GRAY2BGR);

  drawChessboardCorners(cleanBgr, patternSize, cleanCenters, cleanFound);
  drawChessboardCorners(noisyBgr, patternSize, noisyCenters, noisyFound);

  putText(cleanBgr,
          cleanFound ? "clean: centers detected" : "clean: detection failed",
          Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7,
          cleanFound ? Scalar(0, 200, 0) : Scalar(0, 0, 255), 2);

  putText(noisyBgr,
          noisyFound ? "noisy: centers detected" : "noisy: detection failed",
          Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7,
          noisyFound ? Scalar(0, 200, 0) : Scalar(0, 0, 255), 2);

  Mat visualization;
  hconcat(cleanBgr, noisyBgr, visualization);

  cout << "Pattern size: " << patternSize.width << "x" << patternSize.height
       << " centers" << endl;
  cout << "Cell size: " << cellSizePx << " px" << endl;
  cout << "Circle radius: " << radiusPx << " px" << endl;
  cout << "Noise sigma: " << noiseSigma << endl;
  cout << "Seed: " << seed << endl;
  cout << "Clean detection: " << (cleanFound ? "success" : "failure") << endl;
  cout << "Noisy detection: " << (noisyFound ? "success" : "failure") << endl;

  if (!noDisplay) {
    imshow("Circles grid detection (clean | noisy)", visualization);
    waitKey();
  }
}

const char *params =
    "{ help h         |      | print usage }"
    "{ width bw       | 9    | circles grid width (points) }"
    "{ height bh      | 6    | circles grid height (points) }"
    "{ cell_size_px   | 60   | center-to-center spacing in pixels }"
    "{ radius_px      | 14   | circle radius in pixels }"
    "{ noise_sigma    | 20.0 | standard deviation of Gaussian noise }"
    "{ seed           | 42   | random seed used for noise generation }"
    "{ no_display     | false| run without opening a GUI window }";

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example 5: circles grid center detection with added noise.\n");
    parser.printMessage();
    return 0;
  }

  const Size patternSize(parser.get<int>("width"), parser.get<int>("height"));
  const int cellSizePx = parser.get<int>("cell_size_px");
  const int radiusPx = parser.get<int>("radius_px");
  const double noiseSigma = parser.get<double>("noise_sigma");
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

  if (noiseSigma < 0.0) {
    cerr << "noise_sigma must be non-negative." << endl;
    return 1;
  }

  testCirclesPatternDetectionWithNoise(patternSize, cellSizePx, radiusPx,
                                       noiseSigma, seed, noDisplay);
  return 0;
}
