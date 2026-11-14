#include <iostream>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

namespace {

struct MotionEstimate {
  int matches = 0;
  double reprojectionError = 0.0;
  bool valid = false;
};

void calcBoardPoints(const Size &boardSize, const float squareSize,
                     vector<Point3f> &points) {
  points.clear();
  points.reserve(static_cast<size_t>(boardSize.width * boardSize.height));

  for (int y = 0; y < boardSize.height; ++y) {
    for (int x = 0; x < boardSize.width; ++x) {
      points.emplace_back(float(x) * squareSize, float(y) * squareSize, 0.0f);
    }
  }
}

double meanReprojectionError(const vector<Point2f> &source,
                             const vector<Point2f> &target,
                             const Mat &H) {
  if (source.empty() || target.empty() || source.size() != target.size()) {
    return std::numeric_limits<double>::infinity();
  }

  vector<Point2f> projected;
  perspectiveTransform(source, projected, H);

  double total = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    total += norm(projected[i] - target[i]);
  }
  return total / static_cast<double>(projected.size());
}

void estimateRelativeMotionFromFrames(const Mat &prevGray, const Mat &currGray,
                                       MotionEstimate &motion) {
  Ptr<ORB> orb = ORB::create(2000);

  vector<KeyPoint> prevKeypoints, currKeypoints;
  Mat prevDescriptors, currDescriptors;

  orb->detectAndCompute(prevGray, noArray(), prevKeypoints, prevDescriptors);
  orb->detectAndCompute(currGray, noArray(), currKeypoints, currDescriptors);

  if (prevDescriptors.empty() || currDescriptors.empty()) {
    return;
  }

  BFMatcher matcher(NORM_HAMMING);
  vector<vector<DMatch>> matchesKnn;
  matcher.knnMatch(prevDescriptors, currDescriptors, matchesKnn, 2);

  vector<DMatch> goodMatches;
  goodMatches.reserve(matchesKnn.size());
  for (const auto &candidate : matchesKnn) {
    if (candidate.size() < 2) {
      continue;
    }
    if (candidate[0].distance < 0.75 * candidate[1].distance) {
      goodMatches.push_back(candidate[0]);
    }
  }

  if (goodMatches.size() < 8) {
    return;
  }

  vector<Point2f> prevPoints, currPoints;
  prevPoints.reserve(goodMatches.size());
  currPoints.reserve(goodMatches.size());
  for (const auto &match : goodMatches) {
    prevPoints.push_back(prevKeypoints[match.queryIdx].pt);
    currPoints.push_back(currKeypoints[match.trainIdx].pt);
  }

  Mat H = findHomography(prevPoints, currPoints, RANSAC, 3.0);
  if (H.empty()) {
    return;
  }

  motion.matches = (int)goodMatches.size();
  motion.reprojectionError = meanReprojectionError(prevPoints, currPoints, H);
  motion.valid = true;

  vector<Mat> Rs, ts, normals;
  const Mat K = Mat::eye(3, 3, CV_64F);
  int solutions = decomposeHomographyMat(H, K, Rs, ts, normals);
  if (solutions > 0) {
    Mat rvec;
    Rodrigues(Rs[0], rvec);
    cout << "  Relative motion estimate: " << rvec.t() << " | t = "
         << ts[0].t() << endl;
  }
}

bool calibrateCameraFromVideo(const string &videoPath, const Size &boardSize,
                             const float squareSize, const int maxFrames,
                             Mat &cameraMatrix, Mat &distCoeffs,
                             vector<vector<Point3f>> &objectPoints,
                             vector<vector<Point2f>> &imagePoints,
                             vector<int> &frameIds) {
  VideoCapture capture(videoPath);
  if (!capture.isOpened()) {
    cout << "Unable to open video stream: " << videoPath << endl;
    return false;
  }

  vector<Point3f> boardModel;
  calcBoardPoints(boardSize, squareSize, boardModel);

  Mat firstFrame;
  if (!capture.read(firstFrame)) {
    cout << "Unable to read the first frame from the video stream." << endl;
    return false;
  }
  const Size imageSize = firstFrame.size();

  Mat prevGray;
  bool hasPreviousFrame = false;
  int processedFrames = 0;
  int frameId = 0;

  while (processedFrames < maxFrames) {
    Mat frame;
    if (!capture.read(frame)) {
      break;
    }

    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    vector<Point2f> corners;
    bool found = findChessboardCorners(
        gray, boardSize, corners,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

    if (found) {
      cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                   TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30,
                                0.01));

      objectPoints.push_back(boardModel);
      imagePoints.push_back(corners);
      frameIds.push_back(frameId);

      if (hasPreviousFrame) {
        MotionEstimate motion;
        estimateRelativeMotionFromFrames(prevGray, gray, motion);
        if (motion.valid) {
          cout << "Frame " << frameId << ": ORB matches = " << motion.matches
               << ", homography reprojection error = " << motion.reprojectionError
               << endl;
        }
      }

      prevGray = gray.clone();
      hasPreviousFrame = true;
    }

    ++frameId;
    ++processedFrames;
  }

  capture.release();

  if (objectPoints.size() < 10) {
    cout << "Not enough chessboard observations were found in the video stream."
         << endl;
    return false;
  }

  cameraMatrix = Mat::eye(3, 3, CV_64F);
  distCoeffs = Mat::zeros(8, 1, CV_64F);
  vector<Mat> rvecs, tvecs;

  double rmsError = calibrateCamera(objectPoints, imagePoints, imageSize,
                                   cameraMatrix, distCoeffs, rvecs, tvecs);

  cout << "Calibration RMS error: " << rmsError << endl;
  cout << "Recovered camera matrix K:\n" << cameraMatrix << endl;
  return true;
}

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv,
                          "{ help h       | false | print usage }"
                          "{ video v      | 0     | path to a video file or camera id }"
                          "{ width bw     | 9     | chessboard width }"
                          "{ height bh    | 6     | chessboard height }"
                          "{ square_size  | 0.025 | square size in meters }"
                          "{ max_frames n | 60    | maximum number of frames processed }");

  if (parser.get<bool>("help")) {
    parser.printMessage();
    return 0;
  }

  const string videoPath = parser.get<string>("video");
  const Size boardSize(parser.get<int>("width"), parser.get<int>("height"));
  const float squareSize = static_cast<float>(parser.get<double>("square_size"));
  const int maxFrames = parser.get<int>("max_frames");

  Mat cameraMatrix, distCoeffs;
  vector<vector<Point3f>> objectPoints;
  vector<vector<Point2f>> imagePoints;
  vector<int> frameIds;

  bool success = calibrateCameraFromVideo(videoPath, boardSize, squareSize,
                                         maxFrames, cameraMatrix, distCoeffs,
                                         objectPoints, imagePoints, frameIds);

  if (!success) {
    return 1;
  }

  return 0;
}
