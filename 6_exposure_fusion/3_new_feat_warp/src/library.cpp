#include "library.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <vector>

#include "Test/GlobalWarps.h"
#include "Test/ImageGridWarp/ImageGridWarp.h"
#include "Test/Math/Vector.h"
#include "cTJpeg.h"
#include "cTRegressions.h"
#include "common.h"
#include "fuse.h"
#include "imgconversion.h"

#define MAX_IN_OUT_FILES 16

#define WARP_GRID_SIZE_X 11
#define WARP_GRID_SIZE_Y 11

void histEqualization(unsigned char* data, int size) {
  int hist[256];
  unsigned char lut[257];
  memset(hist, 0, sizeof(hist));

  unsigned char* d = data;
  for (int i = size; i > 0; --i) hist[*d++]++;

  float scale = 255.f / (size);
  int sum = 0;
  for (int i = 0; i < 256; ++i) {
    sum += hist[i];
    lut[i] = (int(sum * scale + 0.5f) > 255) ? 255 : int(sum * scale + 0.5f);
  }

  lut[0] = 0;
  d = data;
  for (int i = size; i > 0; --i) *d++ = lut[*d];
}

float getImageHistogramMean(unsigned char* img, int size) {
  int hist[256];
  memset(hist, 0, sizeof(hist));
  for (int l = 0; l < size; l++) hist[img[l]]++;

  float histMean = 0.0f;
  float histSum = 0.0f;
  for (int l = 0; l < 256; l++) {
    histSum += hist[l];
    histMean += hist[l] * l;
  }
  return histMean / histSum;
}

int hdr_cli(int argc, char** argv) {
  int input_count = 0;
  int output_count = 0;
  char* input[MAX_IN_OUT_FILES];
  char* output[MAX_IN_OUT_FILES];

  int prog = 0;
  // 0 - draw features and motion vectors
  // 1 - align images
  // 2 - fuse hdr

  if (!memcmp(argv[1], "draw", 5))
    prog = 0;
  else if (!memcmp(argv[1], "align", 6))
    prog = 1;
  else if (!memcmp(argv[1], "fuse", 5))
    prog = 2;

  int c;
  while ((c = getopt(argc, argv, "i:o:")) != -1) switch (c) {
      case 'i':
        if (optarg != 0) input[input_count++] = strdup(optarg);
        break;
      case 'o':
        if (optarg != 0) output[output_count++] = strdup(optarg);
        break;
      default:;
    }

  printf("Input Files:\n");
  for (int i = 0; i < input_count; ++i) printf("\t%d: %s\n", i, input[i]);
  printf("Output Files:\n");
  for (int i = 0; i < output_count; ++i) printf("\t%d: %s\n", i, output[i]);

  switch (prog) {
    case 0:
      printf("Drawing Features\n");
      break;
    case 1:
      printf("Aligning Frames\n");
      break;
    case 2:
      printf("Fusing HDR\n");
      break;
  }

  int reference = -1;
  hdr_proc_files((const char**)input, (const char**)output, input_count,
                 &reference, prog);

  return 0;
}

int hdr_proc_files(const char** input_files, const char** output_files,
                   int count, int* reference, int prog) {
  long t[2];
  std::vector<cTImg> img(count);
  std::vector<cTJpeg> jpeg(count);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    jpeg[i].fileLoad(input_files[i]);
    img[i] = jpeg[i].getImg();
  }
  t[1] = getuTime();
  LOGI("Load image: %ld us\n", t[1] - t[0]);

  hdr_proc(img.data(), count, reference, prog);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    jpeg[i].fileSave(output_files[i], 100);
    if (prog == 2) break;
  }
  t[1] = getuTime();
  LOGI("Save image: %ld us\n", t[1] - t[0]);

  return 0;
}

int hdr_proc(cTImg* img, int count, int* reference, int prog) {
  long t[2];
  std::vector<cTJpeg> gray(count);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    // img[i].fileLoad(input_files[i]);
    gray[i].create(img[i].width, img[i].height, 1);
    rgb24_to_gray(img[i].buffer, gray[i].getData(), img[i].width, img[i].height,
                  img[i].width, img[i].width);
  }
  t[1] = getuTime();
  LOGI("Convert image: %ld us\n", t[1] - t[0]);

  if (*reference < 0 || *reference >= count) {
    t[0] = getuTime();
    std::vector<float> exposure(count);
    float exposureMean = 0.0f;
    float exposureMin = 255.0f;
    for (int i = 0; i < count; ++i) {
      exposure[i] = getImageHistogramMean(
          gray[i].getData(), gray[i].getWidth() * gray[i].getHeight());
      exposureMean += exposure[i];
    }
    exposureMean /= count;
    for (int i = 0; i < count; ++i) {
      if (fabs(exposure[i] - exposureMean) < exposureMin) {
        exposureMin = fabs(exposure[i] - exposureMean);
        *reference = i;
      }
    }
    // printf("Choosing reference: %d\n", reference);
    t[1] = getuTime();
    LOGI("Choose reference: %ld us\n", t[1] - t[0]);
  }

  std::vector<cv::Mat> mO(count);
  for (int i = 0; i < count; ++i) {
    mO[i] = cv::Mat(img[i].height, img[i].width, CV_8UC3, img[i].buffer);
  }

  std::vector<cv::Mat> m(count);
  for (int i = 0; i < count; ++i) {
    m[i] = cv::Mat(gray[i].getHeight(), gray[i].getWidth(), CV_8UC1,
                   gray[i].getData());
  }

  std::vector<cv::Mat> corners(count);
  t[0] = getuTime();
  cv::goodFeaturesToTrack(m[*reference], corners[*reference], 128, 0.2, 10.0);
  t[1] = getuTime();
  LOGI("Find features: %ld us\n", t[1] - t[0]);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    histEqualization(gray[i].getData(),
                     gray[i].getWidth() * gray[i].getHeight());
  }
  t[1] = getuTime();
  LOGI("Histogram equalization: %ld us\n", t[1] - t[0]);

  std::vector<cv::Mat> status(count);
  std::vector<cv::Mat> err(count);
  t[0] = getuTime();

  int good_points = corners[*reference].rows;
  bool weights[corners[*reference].rows];
  for (int j = 0; j < count; ++j) weights[j] = true;
  // std::cout << "Good points initial:  " << good_points << std::endl;

  for (int i = 0; i < count; ++i) {
    if (i == *reference) continue;
    cv::calcOpticalFlowPyrLK(
        m[*reference], m[i],              // src image,  dst image
        corners[*reference], corners[i],  // src points, dst points
        status[i], err[i], cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10,
                         0.01));

    for (int j = 0; j < corners[i].rows; ++j) {
      float rx =
          corners[*reference].at<float>(j, 0) - corners[i].at<float>(j, 0);
      float ry =
          corners[*reference].at<float>(j, 0) - corners[i].at<float>(j, 0);

      // std::cout << rx*rx+ry*ry << "  " <<
      // 0.05*0.05*(m[i].cols*m[i].cols+m[i].rows*m[i].rows) << std::endl;

      if (weights[j] &&
          ((rx * rx + ry * ry) >
           0.03 * 0.03 * (m[i].cols * m[i].cols + m[i].rows * m[i].rows))) {
        // weights[j]=false;
        // good_points--;
      }
    }
  }

  std::cout << "Test new detector settings." << std::endl;
  std::cout << "Good points:  " << good_points << std::endl;

  t[1] = getuTime();
  LOGI("KLT: %ld us\n", t[1] - t[0]);

  if (prog == 0) {
    t[0] = getuTime();
    for (int i = 0; i < count; ++i) {
      for (int p = 0; p < corners[i].rows; ++p) {
        cv::circle(
            mO[i],
            cv::Point2f(corners[i].at<float>(p, 0), corners[i].at<float>(p, 1)),
            5, cv::Scalar(0, 0, 0), 2);
        cv::circle(
            mO[i],
            cv::Point2f(corners[i].at<float>(p, 0), corners[i].at<float>(p, 1)),
            2, cv::Scalar(255, 0, 0), 3);
      }

      if (i != *reference) {
        for (int p = 0; p < corners[i].rows; ++p) {
          cv::line(mO[i],
                   cv::Point2f(corners[i].at<float>(p, 0),
                               corners[i].at<float>(p, 1)),
                   cv::Point2f(corners[*reference].at<float>(p, 0),
                               corners[*reference].at<float>(p, 1)),
                   cv::Scalar(255, 0, 0), 2);
        }
      }
    }
    t[1] = getuTime();
    LOGI("Draw: %ld us\n", t[1] - t[0]);
  } else if (prog == 1 || prog == 2) {
    std::vector<std::array<double, 9>> coeff(count);

    std::vector<std::vector<cv::Point2f>> vcorners(count);
    std::vector<std::vector<unsigned char>> vstatus(count);
    for (int i = 0; i < count; ++i) {
      vcorners[i] = std::vector<cv::Point2f>(corners[i]);
      vstatus[i] = std::vector<unsigned char>(status[i]);
    }

    t[0] = getuTime();

    Test::GyroVStab::GlobalWarps<Test::Math::Vector<float, 2>, float> grid;
    Test::Math::Vector<float, 2>* temp_corner =
        (Test::Math::Vector<float, 2>*)corners[*reference].data;
    Test::Math::Vector<float, 2> mesh_grid[WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y];
    Test::Math::Vector<float, 2> reference_corner[good_points];
    for (int j = 0, j1 = 0; j < corners[*reference].rows; j++) {
      if (weights[j]) {
        reference_corner[j1] = *(temp_corner + j);
        reference_corner[j1].x() =
            2.0 * (reference_corner[j1].x() / img[*reference].width) - 1.0;
        reference_corner[j1].y() =
            2.0 * (reference_corner[j1].y() / img[*reference].height) - 1.0;
        j1++;
        // std::cout << j1 << std::endl;
        // std::cout << reference_corner[j1].x() << "  " <<
        // reference_corner[j1].y() << std::endl;
      }
      std::cout << j1 << std::endl;
    }

    grid.nowarp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);

    for (int i = 0; i < count; ++i) {
      if (i == *reference) continue;

      Test::Math::Vector<float, 2>* temp_corner =
          (Test::Math::Vector<float, 2>*)corners[i].data;
      Test::Math::Vector<float, 2> sample_corner[good_points];
      for (int j = 0, j1 = 0; j < corners[i].rows; j++) {
        if (weights[j]) {
          sample_corner[j1] = *(temp_corner + j);
          sample_corner[j1].x() =
              2.0 * (sample_corner[j1].x() / img[i].width) - 1.0;
          sample_corner[j1].y() =
              2.0 * (sample_corner[j1].y() / img[i].height) - 1.0;
          j1++;
          // std::cout << j1 << std::endl;
          // std::cout << sample_corner[j1].x() << "  " << sample_corner[j1].y()
          // << std::endl;
        }
        std::cout << j1 << std::endl;
      }

      grid.regPerspRobust(sample_corner, reference_corner, good_points);

      grid.warp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);

      cTJpeg temp;
      temp.create(img[i].width, img[i].height, img[i].channels);

      cTImg tempimg = temp.getImg();
      for (int j = 0; j < img[i].width * img[i].height * img[i].channels; j++)
        *(tempimg.buffer + j) = 0;

      Test::Math::Vector<float, 2>
          mesh_grid1[WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y];
      for (int j1 = 0; j1 < WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y; j1++) {
        float temp_x = mesh_grid[j1].x();
        float temp_y = mesh_grid[j1].y();
        mesh_grid1[j1].x() = (1.0 + temp_x) * img[i].width * 0.5;
        mesh_grid1[j1].y() = (1.0 + temp_y) * img[i].height * 0.5;
      }

      imageGridWarpBilinearI(

          // Image
          img[i].buffer, img[i].width, img[i].height, img[i].width,
          temp.getData(), img[i].width, img[i].height, img[i].width,

          // Warped Grid
          WARP_GRID_SIZE_X - 1, WARP_GRID_SIZE_Y - 1, &mesh_grid1[0].x(),
          sizeof(Test::Math::Vector<float, 2>) /
              sizeof(mesh_grid[0].x()),  // X, x stride
          &mesh_grid1[0].y(),
          sizeof(Test::Math::Vector<float, 2>) /
              sizeof(mesh_grid[0].y()),  // Y, y stride
          1);

      for (int j = 0; j < img[i].width * img[i].height * img[i].channels; j++)
        *(img[i].buffer + j) = *(tempimg.buffer + j);
    }
    t[1] = getuTime();
    LOGI("Regression + Warping: %ld us\n", t[1] - t[0]);

    if (prog == 2) {
      std::vector<unsigned short*> img_sh(count);
      cTJpeg img_out;
      img_out.create(mO[0].cols, mO[0].rows, 2);

      // Convert the images to yuv422
      t[0] = getuTime();
      for (int i = 0; i < count; ++i) {
        rgb24_to_yuv422(mO[i].data, mO[i].data, mO[i].cols, mO[i].rows,
                        mO[i].cols, mO[i].cols);
        img_sh[i] = (unsigned short*)mO[i].data;
      }
      t[1] = getuTime();
      LOGI("RGB to YUV: %ld us\n", t[1] - t[0]);

      // Fuse
      t[0] = getuTime();
      test_mfhdr_process(img_sh.data(), count,
                         (unsigned short*)img_out.getData(), img_out.getWidth(),
                         img_out.getHeight());
      t[1] = getuTime();
      LOGI("Fusion: %ld us\n", t[1] - t[0]);

      // Convert back to RGB
      t[0] = getuTime();
      yuv422_to_rgb24(img_out.getData(), mO[0].data, mO[0].cols, mO[0].rows,
                      mO[0].cols, mO[0].cols);
      t[1] = getuTime();
      LOGI("YUV to RGB: %ld us\n", t[1] - t[0]);
    }
  }

  return 0;
}
