#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace {

Mat generateChessboardImage(const Size& patternSize, int squareSizePx) {
  const int boardSquaresX = patternSize.width + 1;
  const int boardSquaresY = patternSize.height + 1;
  Mat board(boardSquaresY * squareSizePx, boardSquaresX * squareSizePx, CV_8UC1,
            Scalar(0));

  for (int y = 0; y < boardSquaresY; ++y) {
    for (int x = 0; x < boardSquaresX; ++x) {
      if ((x + y) % 2 == 0) {
        rectangle(board, Rect(x * squareSizePx, y * squareSizePx, squareSizePx,
                              squareSizePx),
                  Scalar(255), FILLED);
      }
    }
  }

  return board;
}

Mat addGaussianNoise(const Mat& gray, double sigma, uint64 seed) {
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

bool detectChessboardCorners(const Mat& gray, const Size& patternSize,
                             vector<Point2f>& corners) {
  bool found = findChessboardCorners(
      gray, patternSize, corners,
      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

  if (found) {
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                 TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30,
                              0.01));
  }

  return found;
}

void testChessboardDetectionWithNoise(const Size& patternSize, int squareSizePx,
                                      double noiseSigma, uint64 seed) {
  Mat cleanGray = generateChessboardImage(patternSize, squareSizePx);
  Mat noisyGray = addGaussianNoise(cleanGray, noiseSigma, seed);

  vector<Point2f> cleanCorners;
  vector<Point2f> noisyCorners;

  bool cleanFound = detectChessboardCorners(cleanGray, patternSize, cleanCorners);
  bool noisyFound = detectChessboardCorners(noisyGray, patternSize, noisyCorners);

  Mat cleanBgr, noisyBgr;
  cvtColor(cleanGray, cleanBgr, COLOR_GRAY2BGR);
  cvtColor(noisyGray, noisyBgr, COLOR_GRAY2BGR);

  drawChessboardCorners(cleanBgr, patternSize, cleanCorners, cleanFound);
  drawChessboardCorners(noisyBgr, patternSize, noisyCorners, noisyFound);

  putText(cleanBgr,
          cleanFound ? "clean: corners detected" : "clean: detection failed",
          Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8,
          cleanFound ? Scalar(0, 200, 0) : Scalar(0, 0, 255), 2);

  putText(noisyBgr,
          noisyFound ? "noisy: corners detected" : "noisy: detection failed",
          Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8,
          noisyFound ? Scalar(0, 200, 0) : Scalar(0, 0, 255), 2);

  Mat visualization;
  hconcat(cleanBgr, noisyBgr, visualization);

  cout << "Pattern size: " << patternSize.width << "x" << patternSize.height
       << " corners" << endl;
  cout << "Square size: " << squareSizePx << " px" << endl;
  cout << "Noise sigma: " << noiseSigma << endl;
  cout << "Seed: " << seed << endl;
  cout << "Clean detection: " << (cleanFound ? "success" : "failure") << endl;
  cout << "Noisy detection: " << (noisyFound ? "success" : "failure") << endl;

  imshow("Chessboard corner detection (clean | noisy)", visualization);
  waitKey();
}

const char* params =
    "{ help h         |      | print usage }"
    "{ width bw       | 9    | chessboard width (inner corners) }"
    "{ height bh      | 6    | chessboard height (inner corners) }"
    "{ square_size_px | 60   | chessboard square size in pixels }"
    "{ noise_sigma    | 20.0 | standard deviation of Gaussian noise }"
    "{ seed           | 42   | random seed used for noise generation }";

}  // namespace

int main(int argc, char* argv[]) {
  CommandLineParser parser(argc, argv, params);
  if (parser.has("help")) {
    parser.about(
        "Code for homography tutorial.\n"
        "Example 4: chessboard corner detection with added noise.\n");
    parser.printMessage();
    return 0;
  }

  Size patternSize(parser.get<int>("width"), parser.get<int>("height"));
  int squareSizePx = parser.get<int>("square_size_px");
  double noiseSigma = parser.get<double>("noise_sigma");
  uint64 seed = static_cast<uint64>(parser.get<int>("seed"));

  if (patternSize.width <= 0 || patternSize.height <= 0) {
    cerr << "Pattern dimensions must be positive." << endl;
    return 1;
  }

  if (squareSizePx <= 0) {
    cerr << "square_size_px must be positive." << endl;
    return 1;
  }

  if (noiseSigma < 0.0) {
    cerr << "noise_sigma must be non-negative." << endl;
    return 1;
  }

  testChessboardDetectionWithNoise(patternSize, squareSizePx, noiseSigma, seed);
  return 0;
}
