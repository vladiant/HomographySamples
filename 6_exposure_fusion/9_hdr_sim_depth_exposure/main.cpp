#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

// cTImg wrapper used by the HDR fusing routine
#include <cTImg.h>

// HDR fusing algorithm interface
#include <rhdr_interface.h>

using namespace cv;
using namespace std;

// Create a stack of 8-bit RGB exposures from a single high dynamic range image.
void createExposures(const Mat& img, vector<Mat>& images,
                     vector<unsigned long>& exposures,
                     vector<unsigned int>& gains);

int main(int argc, char** argv) {
  cout << "\nMulti Frame High Dynamic Range simulator (depth exposure)\n\n";

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
    cout << "8-bit depth image is not a high dynamic range source.\n";
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
  vector<unsigned long> exposures;
  vector<unsigned int> gains;
  createExposures(img, images, exposures, gains);

  const int input_count = static_cast<int>(images.size());
  if (input_count < 2) {
    cout << "Not enough exposures were created for fusion.\n";
    return EXIT_FAILURE;
  }

  // Describe the generated exposures with the cTImg wrapper expected by the
  // fusing routine. OpenCV loads BGR, the routine works with RGB.
  vector<cTImg> frames(input_count);
  for (int i = 0; i < input_count; ++i) {
    cvtColor(images[i], images[i], COLOR_BGR2RGB);
    frames[i].width = static_cast<unsigned short>(images[i].cols);
    frames[i].height = static_cast<unsigned short>(images[i].rows);
    frames[i].channels = static_cast<unsigned short>(images[i].channels());
    frames[i].buffer = images[i].data;
  }

  Mat out_image(images[0].size(), CV_8UC3, Scalar(0, 0, 0));

  // Fuse the exposures directly. No alignment is needed because every exposure
  // originates from the same source image.
  const int ref_image = input_count / 2;
  hdr_fuse(frames.data(), reinterpret_cast<char*>(out_image.data),
           exposures.data(), gains.data(), input_count, ref_image);

  cvtColor(out_image, out_image, COLOR_RGB2BGR);
  imwrite("hdr_result.png", out_image);
  cout << "Output image saved to hdr_result.png\n";

  return 0;
}

// Robust white point: the value below which the given percentile of samples
// fall, so a few very bright pixels do not dominate the normalization.
static double percentileValue(const Mat& img, double percentile) {
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

void createExposures(const Mat& img, vector<Mat>& images,
                     vector<unsigned long>& exposures,
                     vector<unsigned int>& gains) {
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
  const unsigned long baseExposureUs = 1000;  // microseconds for unity time
  const unsigned int unityGain = 256;         // U16Q8 multiplier for 1.0x

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
    exposures.push_back(
        static_cast<unsigned long>(baseExposureUs * exposureTime));
    gains.push_back(unityGain);

    cout << "Created exposure with time: " << exposureTime << "\n";
  }
}
