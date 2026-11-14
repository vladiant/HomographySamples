#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

// Robust white point: the value below which the given percentile of samples
// fall, so a few very bright pixels do not dominate the normalization.
static double percentileValue(const Mat &img, double percentile) {
  Mat values;
  img.reshape(1).convertTo(values, CV_32F);
  vector<float> data(values.begin<float>(), values.end<float>());
  if (data.empty()) {
    return 0.0;
  }
  size_t index = static_cast<size_t>((percentile / 100.0) * (data.size() - 1));
  nth_element(data.begin(), data.begin() + index, data.end());
  return data[index];
}

// Split a single high dynamic range image into a stack of 8-bit exposures.
static void createExposures(const Mat &img, vector<Mat> &images,
                            vector<float> &times) {
  double minVal{};
  double maxVal{};
  // reshape to single channel so minMaxLoc works on multi-channel images.
  minMaxLoc(img.reshape(1), &minVal, &maxVal);
  cout << "Min value: " << minVal << ", Max value: " << maxVal << "\n";

  // Use a high percentile as the white point so bright highlights do not
  // collapse the rest of the scene to black.
  double whitePoint = percentileValue(img, 99.0);
  if (whitePoint <= minVal) {
    whitePoint = maxVal;
  }
  double range = whitePoint - minVal;
  if (range <= 0.0) {
    range = 1.0;
  }
  cout << "White point (99th pct): " << whitePoint << "\n";
  Mat normalized;
  img.convertTo(normalized, CV_32F, 1.0 / range, -minVal / range);

  // Brighten with increasing exposure times, clipping highlights so each
  // exposure reveals a different tonal range.
  const vector<float> exposureTimes{0.25f, 0.5f, 1.0f, 2.0f, 4.0f};
  const double invGamma = 1.0 / 2.2;  // gamma-encode linear data for display
  for (float exposureTime : exposureTimes) {
    Mat scaled;
    normalized.convertTo(scaled, CV_32F, exposureTime);
    threshold(scaled, scaled, 1.0, 1.0, THRESH_TRUNC);
    scaled = cv::max(scaled, 0.0);

    Mat encoded;
    pow(scaled, invGamma, encoded);

    Mat exposure;
    encoded.convertTo(exposure, CV_8U, 255.0);
    images.push_back(exposure);
    times.push_back(exposureTime);

    cout << "Created exposure with time: " << exposureTime << "\n";
  }
}

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Usage : " << argv[0] << " image_filename\n";
    return EXIT_FAILURE;
  }

  Mat img;
  if ((img = imread(argv[1], IMREAD_COLOR | IMREAD_ANYDEPTH)).empty()) {
    cout << "Error reading : " << argv[1] << "\n";
    return EXIT_FAILURE;
  }

  if (img.depth() == CV_8U || img.depth() == CV_8S) {
    cout << "8-bit depth image cannot be split.\n";
    return EXIT_FAILURE;
  }

  switch (img.depth()) {
    case CV_16F:
      cout << "16-bit float image\n";
      break;
    case CV_32F:
      cout << "32-bit float image\n";
      break;
    case CV_64F:
      cout << "64-bit float image\n";
      break;
    case CV_16U:
      cout << "16-bit unsigned integer image\n";
      break;
    case CV_16S:
      cout << "16-bit signed integer image\n";
      break;
    case CV_32S:
      cout << "32-bit signed integer image\n";
      break;
    default:
      cout << "Unsupported image depth: " << img.depth() << "\n";
      return EXIT_FAILURE;
  }

  vector<Mat> images;
  vector<float> times;
  createExposures(img, images, times);

  for (size_t i = 0; i < images.size(); ++i) {
    string name = "split_" + to_string(i) + ".png";
    imwrite(name, images[i]);
    cout << "Saved " << name << " (exposure time " << times[i] << ")\n";
  }

  return 0;
}

