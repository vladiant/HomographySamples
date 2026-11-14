#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

extern "C" {
#include "rhdr.h"
}

// Referenced by the rhdr library for debugging hooks.
extern "C" void blank(void) {}

using namespace cv;
using namespace std;

// Input gamma that maps the 8-bit exposure values onto the HDR working range.
static uint16_t const_gamma[256] = {
    0,   4,   8,   12,  15,  18,  21,  24,  26,  29,  32,  34,  36,  38,  41,
    43,  45,  47,  49,  51,  52,  54,  56,  58,  60,  61,  63,  65,  66,  68,
    69,  71,  72,  74,  75,  77,  78,  80,  81,  82,  84,  85,  87,  88,  89,
    90,  92,  93,  94,  96,  97,  98,  99,  100, 102, 103, 104, 105, 106, 107,
    109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 121, 122, 123, 124,
    125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 135, 136, 137, 138,
    139, 140, 141, 142, 143, 144, 145, 146, 147, 147, 148, 149, 150, 151, 152,
    153, 154, 154, 155, 156, 157, 158, 159, 160, 160, 161, 162, 163, 164, 164,
    165, 166, 167, 168, 169, 169, 170, 171, 172, 172, 173, 174, 175, 176, 176,
    177, 178, 179, 179, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 188,
    188, 189, 190, 191, 191, 192, 193, 193, 194, 195, 196, 196, 197, 198, 198,
    199, 200, 200, 201, 202, 202, 203, 204, 205, 205, 206, 207, 207, 208, 209,
    209, 210, 211, 211, 212, 213, 213, 214, 215, 215, 216, 216, 217, 218, 218,
    219, 220, 220, 221, 222, 222, 223, 224, 224, 225, 225, 226, 227, 227, 228,
    228, 229, 230, 230, 231, 232, 232, 233, 233, 234, 235, 235, 236, 236, 237,
    238, 238, 239, 239, 240, 241, 241, 242, 242, 243, 244, 244, 245, 245, 246,
    246, 247, 248, 248, 249, 249, 250, 250, 251, 252, 252, 253, 253, 254, 254,
    255};

// Output gamma applied to the fused HDR result before saving.
static uint16_t gamma_hdr_temp[256] = {
#include "hdr_gamma_temp.txt"
};

#define SMEAR_CORE_SIZE (9)
static uint16_t rhdr_smear_core[SMEAR_CORE_SIZE * SMEAR_CORE_SIZE] = {
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000,
};

static rhdr_noise_vs_gain_t rhdr_noise_vs_gain[] = {
    {36, (12288 >> 4)}, {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400}, {12, 400}, {12, 400}, {12, 400}, {12, 400},
    {12, 400},          {12, 400},
};

static rhdr_tune_data_t rhdr_tune_data = {
    {
#include "pix_val_weight.txt"
    },
    0,                                                   // mov_dif_thr_const
    100,                                                 // mov_dif_thr_mult
    0,                                                   // smear_core_size
    rhdr_smear_core,                                     // smear_core
    sizeof(rhdr_noise_vs_gain) / sizeof(rhdr_noise_vs_gain[0]),
    rhdr_noise_vs_gain,                                  // noise_vs_gain
    2,                                                   // dark_snr_thr
};

static void* rhdr_ctx = NULL;
static rhdr_in_data_t rhdr_in;

// Create a stack of 8-bit RGB exposures from a single high dynamic range image.
void createExposures(const Mat& img, vector<Mat>& images,
                     vector<uint32_t>& exposures, vector<uint16_t>& gains);

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
  vector<uint32_t> exposures;
  vector<uint16_t> gains;
  createExposures(img, images, exposures, gains);

  const int input_count = static_cast<int>(images.size());
  if (input_count < 2) {
    cout << "Not enough exposures were created for fusion.\n";
    return EXIT_FAILURE;
  }

  const uint16_t sx = static_cast<uint16_t>(images[0].cols);
  const uint16_t sy = static_cast<uint16_t>(images[0].rows);

  // Fill the HDR input structure.
  rhdr_in.enable = 1;
  rhdr_in.img_count = static_cast<uint8_t>(input_count);
  rhdr_in.raw_bitsperpix = 16;  // needed for RGB HDR
  rhdr_in.img_sx = sx;
  rhdr_in.img_sy = sy;
  rhdr_in.img_ppln = sx;
  rhdr_in.data_pedestal = 0;
  rhdr_in.tune_data = &rhdr_tune_data;
  rhdr_in.ref_image = static_cast<uint8_t>(input_count / 2);
  memcpy(rhdr_in.in_gamma, const_gamma, sizeof(uint16_t) * 256);

  // Convert each 8-bit BGR exposure into an interleaved 16-bit RGB buffer.
  for (int i = 0; i < input_count; ++i) {
    rhdr_in.img_exps_us[i] = exposures[i];
    rhdr_in.img_gains[i] = gains[i];

    uint16_t* buffer =
        static_cast<uint16_t*>(malloc(sizeof(uint16_t) * sx * sy * 3));
    if (buffer == NULL) {
      cout << "Can not allocate memory for input image\n";
      return EXIT_FAILURE;
    }

    for (int row = 0; row < sy; ++row) {
      for (int col = 0; col < sx; ++col) {
        const Vec3b& pixel = images[i].at<Vec3b>(row, col);
        uint16_t* out = buffer + 3 * (row * sx + col);
        out[0] = pixel[2];  // R
        out[1] = pixel[1];  // G
        out[2] = pixel[0];  // B
      }
    }
    rhdr_in.ptr_imgs[i] = buffer;
  }

  rhdr_in.out_rgb =
      static_cast<uint16_t*>(malloc(sizeof(uint16_t) * sx * sy * 3));
  if (rhdr_in.out_rgb == NULL) {
    cout << "Can not allocate memory for output image\n";
    return EXIT_FAILURE;
  }

  // Fuse the exposures. No alignment is needed because every exposure
  // originates from the same source image.
  int err = 0;
  err |= test_rhdr_create((void**)(&rhdr_ctx));
  err |= test_rhdr_process(rhdr_ctx, &rhdr_in);
  err |= test_rhdr_delete(rhdr_ctx);
  if (err) {
    cout << "rhdr ERROR\n";
  }

  // Apply the output gamma and build an 8-bit BGR image for saving.
  Mat out_image(sy, sx, CV_8UC3);
  for (int row = 0; row < sy; ++row) {
    for (int col = 0; col < sx; ++col) {
      Vec3b& pixel = out_image.at<Vec3b>(row, col);
      const uint16_t* rgb = rhdr_in.out_rgb + 3 * (row * sx + col);
      for (int c = 0; c < 3; ++c) {
        uint32_t slp = 0;
        uint32_t r1 = static_cast<uint32_t>(rgb[c])
                      << (16 - RHDR_OUT_BITS_PER_PIX);
        if ((r1 >> 8) < 255) {
          slp = gamma_hdr_temp[(r1 >> 8) + 1] - gamma_hdr_temp[(r1 >> 8)];
        }
        r1 = (static_cast<uint32_t>(gamma_hdr_temp[r1 >> 8]) << 6) +
             (slp << 6) * (r1 & ((1 << 8) - 1)) / (1 << 8);
        // BGR order for OpenCV; gamma output is 16-bit, keep the top 8 bits.
        pixel[2 - c] = static_cast<uchar>(r1 >> 8);
      }
    }
  }

  for (int i = 0; i < input_count; ++i) {
    free(rhdr_in.ptr_imgs[i]);
  }
  free(rhdr_in.out_rgb);

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
                     vector<uint32_t>& exposures, vector<uint16_t>& gains) {
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
  const uint32_t baseExposureUs = 1000;  // microseconds for unity time
  const uint16_t unityGain = 256;        // U16Q8 multiplier for 1.0x

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
    exposures.push_back(static_cast<uint32_t>(baseExposureUs * exposureTime));
    gains.push_back(unityGain);

    cout << "Created exposure with time: " << exposureTime << "\n";
  }
}
