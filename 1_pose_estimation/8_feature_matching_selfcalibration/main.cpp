#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

namespace {

// Lowe ratio test on the two nearest neighbours of each descriptor.
void matchFeatures(const Mat &descriptors1, const Mat &descriptors2,
                   const vector<KeyPoint> &keypoints1,
                   const vector<KeyPoint> &keypoints2,
                   vector<Point2f> &points1, vector<Point2f> &points2) {
  points1.clear();
  points2.clear();
  if (descriptors1.empty() || descriptors2.empty()) {
    return;
  }

  BFMatcher matcher(NORM_HAMMING);
  vector<vector<DMatch>> knnMatches;
  matcher.knnMatch(descriptors1, descriptors2, knnMatches, 2);

  for (const auto &candidate : knnMatches) {
    if (candidate.size() < 2) {
      continue;
    }
    if (candidate[0].distance < 0.75 * candidate[1].distance) {
      points1.push_back(keypoints1[candidate[0].queryIdx].pt);
      points2.push_back(keypoints2[candidate[0].trainIdx].pt);
    }
  }
}

// Bougnoux closed-form focal length from the fundamental matrix, assuming the
// principal point is shared by both views and square pixels.
bool focalFromFundamental(const Mat &F, const Point2d &principalPoint,
                          double &focal) {
  Mat Fd;
  F.convertTo(Fd, CV_64F);

  // Epipole in the second image is the left null vector of F.
  Mat w, u, vt;
  SVDecomp(Fd, w, u, vt, SVD::FULL_UV);
  const Mat epipole2 = u.col(2);

  const Mat p =
      (Mat_<double>(3, 1) << principalPoint.x, principalPoint.y, 1.0);

  const Mat skewEpipole = (Mat_<double>(3, 3) << 0.0, -epipole2.at<double>(2),
                           epipole2.at<double>(1), epipole2.at<double>(2), 0.0,
                           -epipole2.at<double>(0), -epipole2.at<double>(1),
                           epipole2.at<double>(0), 0.0);

  const Mat diag110 =
      (Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

  const Mat numeratorLeft = p.t() * skewEpipole * diag110 * Fd * p;
  const Mat numeratorRight = p.t() * Fd.t() * p;
  const Mat denominator = p.t() * skewEpipole * diag110 * Fd * diag110 * Fd.t() * p;

  const double den = denominator.at<double>(0, 0);
  if (abs(den) < 1e-12) {
    return false;
  }

  const double focalSquared =
      -(numeratorLeft.at<double>(0, 0) * numeratorRight.at<double>(0, 0)) / den;
  if (focalSquared <= 0.0) {
    return false;
  }

  focal = sqrt(focalSquared);
  return true;
}

bool selfCalibrateFromVideo(const string &videoPath, const int maxFrames,
                            const int stride, const int minInliers,
                            Mat &cameraMatrix) {
  VideoCapture capture(videoPath);
  if (!capture.isOpened()) {
    cout << "Unable to open video stream: " << videoPath << endl;
    return false;
  }

  Ptr<ORB> orb = ORB::create(3000);

  vector<Mat> grayFrames;
  vector<vector<KeyPoint>> keypointsPerFrame;
  vector<Mat> descriptorsPerFrame;

  Size imageSize(0, 0);
  int frameCount = 0;
  while (frameCount < maxFrames) {
    Mat frame;
    if (!capture.read(frame)) {
      break;
    }
    if (imageSize.width == 0) {
      imageSize = frame.size();
    }

    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    vector<KeyPoint> keypoints;
    Mat descriptors;
    orb->detectAndCompute(gray, noArray(), keypoints, descriptors);

    grayFrames.push_back(gray);
    keypointsPerFrame.push_back(keypoints);
    descriptorsPerFrame.push_back(descriptors);
    ++frameCount;
  }
  capture.release();

  if (static_cast<int>(grayFrames.size()) <= stride) {
    cout << "Not enough frames were read to form a pair with the given stride."
         << endl;
    return false;
  }

  const Point2d principalPoint(imageSize.width / 2.0, imageSize.height / 2.0);

  vector<double> focalEstimates;
  for (size_t i = 0; i + stride < grayFrames.size(); i += stride) {
    const size_t j = i + stride;

    vector<Point2f> points1, points2;
    matchFeatures(descriptorsPerFrame[i], descriptorsPerFrame[j],
                  keypointsPerFrame[i], keypointsPerFrame[j], points1, points2);

    if (points1.size() < 15) {
      continue;
    }

    vector<uchar> inlierMask;
    Mat F = findFundamentalMat(points1, points2, FM_RANSAC, 1.0, 0.999,
                               inlierMask);
    if (F.empty() || F.rows != 3) {
      continue;
    }

    const int inliers =
        countNonZero(inlierMask) > 0 ? countNonZero(inlierMask) : 0;
    if (inliers < minInliers) {
      continue;
    }

    double focal = 0.0;
    if (!focalFromFundamental(F, principalPoint, focal)) {
      continue;
    }

    // Reject degenerate focal lengths that are physically implausible.
    if (focal < 0.2 * imageSize.width || focal > 5.0 * imageSize.width) {
      continue;
    }

    focalEstimates.push_back(focal);
    cout << "Pair (" << i << ", " << j << "): inliers = " << inliers
         << ", focal = " << focal << endl;
  }

  if (focalEstimates.empty()) {
    cout << "Could not recover a stable focal length. Make sure the scene is "
            "non-planar and the camera translates."
         << endl;
    return false;
  }

  sort(focalEstimates.begin(), focalEstimates.end());
  const double medianFocal = focalEstimates[focalEstimates.size() / 2];

  cameraMatrix = (Mat_<double>(3, 3) << medianFocal, 0.0, principalPoint.x, 0.0,
                  medianFocal, principalPoint.y, 0.0, 0.0, 1.0);

  cout << "\nEstimated focal length (median of " << focalEstimates.size()
       << " pairs): " << medianFocal << endl;
  cout << "Recovered camera matrix K:\n" << cameraMatrix << endl;

  // Validation: estimate the essential matrix with the recovered K and recover
  // the relative pose of a representative pair.
  for (size_t i = 0; i + stride < grayFrames.size(); i += stride) {
    const size_t j = i + stride;
    vector<Point2f> points1, points2;
    matchFeatures(descriptorsPerFrame[i], descriptorsPerFrame[j],
                  keypointsPerFrame[i], keypointsPerFrame[j], points1, points2);
    if (points1.size() < 15) {
      continue;
    }

    vector<uchar> inlierMask;
    Mat E = findEssentialMat(points1, points2, cameraMatrix, RANSAC, 0.999, 1.0,
                             inlierMask);
    if (E.empty() || E.rows != 3) {
      continue;
    }

    Mat R, t;
    const int poseInliers =
        recoverPose(E, points1, points2, cameraMatrix, R, t, inlierMask);
    Mat rvec;
    Rodrigues(R, rvec);
    cout << "\nValidation on pair (" << i << ", " << j
         << "): pose inliers = " << poseInliers << endl;
    cout << "  relative rotation (rvec): " << rvec.t() << endl;
    cout << "  relative translation (unit): " << t.t() << endl;
    break;
  }

  return true;
}

}  // namespace

int main(int argc, char *argv[]) {
  CommandLineParser parser(
      argc, argv,
      "{ help h       | false | print usage }"
      "{ video v      | 0     | path to a video file or camera id }"
      "{ max_frames n | 120   | maximum number of frames processed }"
      "{ stride s     | 5     | frame spacing between the two views of a pair }"
      "{ min_inliers  | 50    | minimum RANSAC inliers to accept a pair }");

  if (parser.get<bool>("help")) {
    parser.printMessage();
    return 0;
  }

  const string videoPath = parser.get<string>("video");
  const int maxFrames = parser.get<int>("max_frames");
  const int stride = max(1, parser.get<int>("stride"));
  const int minInliers = parser.get<int>("min_inliers");

  Mat cameraMatrix;
  const bool success =
      selfCalibrateFromVideo(videoPath, maxFrames, stride, minInliers,
                             cameraMatrix);

  return success ? 0 : 1;
}
