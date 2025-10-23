#include <math.h>
#include <stdio.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <vector>

#include "Test/GlobalWarps.h"
#include "Test/ImageGridWarp/ImageGridWarp.h"
#include "Test/Math/Vector.h"
#include "cTFeatureSegmentContainer.h"
#include "cTJpeg.h"
#include "cTMostFeatureDetector.h"
#include "cTRegressions.h"
#include "common.h"
#include "fuse.h"
#include "imgconversion.h"
#include "library.h"

#define MAX_IN_OUT_FILES 16

#define WARP_GRID_SIZE_X 16
#define WARP_GRID_SIZE_Y 12
#define SEG_WIDTH 16
#define SEG_HEIGHT 12
#define THRESHOLD 20
#define MAX_DIST 30

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
  for (int l = size; l > 0; --l) hist[img[l]]++;

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
  cTImg img[count];
  cTJpeg jpeg[count];

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    jpeg[i].fileLoad(input_files[i]);
    img[i] = jpeg[i].getImg();
  }
  t[1] = getuTime();
  LOGI("Load image: %d us\n", t[1] - t[0]);

  hdr_proc(img, count, reference, prog);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    jpeg[i].fileSave(output_files[i], 100);
    if (prog == 2) break;
  }
  t[1] = getuTime();
  LOGI("Save image: %d us\n", t[1] - t[0]);
}

int hdr_proc(cTImg* img, int count, int* reference, int prog) {
  long t[2];
  cTJpeg gray[count];
  float exposure[count];

  // for a set of THREE images !!!
  int under_exp, over_exp;

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    // img[i].fileLoad(input_files[i]);
    gray[i].create(img[i].width, img[i].height, 1);
    rgb24_to_gray(img[i].buffer, gray[i].getData(), img[i].width, img[i].height,
                  img[i].width, img[i].width);
  }
  t[1] = getuTime();
  LOGI("Convert image: %d us\n", t[1] - t[0]);

  if (*reference < 0 || *reference >= count) {
    t[0] = getuTime();
    // float exposure[count];
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
    LOGI("Choose reference: %d us\n", t[1] - t[0]);
  }

  // for a set of THREE images !!!
  if (*reference == 0) {
    if (exposure[1] > exposure[2]) {
      over_exp = 1;
      under_exp = 2;
    } else {
      over_exp = 2;
      under_exp = 1;
    }
  }

  if (*reference == 1) {
    if (exposure[0] > exposure[2]) {
      over_exp = 0;
      under_exp = 2;
    } else {
      over_exp = 2;
      under_exp = 0;
    }
  }

  if (*reference == 2) {
    if (exposure[1] > exposure[0]) {
      over_exp = 1;
      under_exp = 0;
    } else {
      over_exp = 0;
      under_exp = 1;
    }
  }

  cv::Mat mO1[count];
  cv::Mat mO2[count];
  for (int i = 0; i < count; ++i) {
    cv::Mat tempmat =
        cv::Mat(img[i].height, img[i].width, CV_8UC3, img[i].buffer);
    // mO1[i] = cv::Mat(img[i].height, img[i].width, CV_8UC3, img[i].buffer);
    mO1[i] = tempmat.clone();
    // mO2[i] = cv::Mat(img[i].height, img[i].width, CV_8UC3, img[i].buffer);
    mO2[i] = tempmat.clone();
  }

  cv::Mat m[count];
  for (int i = 0; i < count; ++i) {
    m[i] = cv::Mat(gray[i].getHeight(), gray[i].getWidth(), CV_8UC1,
                   gray[i].getData());
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////

  // cv::Mat corners[count];
  std::vector<cv::Point2f> corners[count];
  std::vector<cv::Point2f> corners1[count];
  std::vector<cv::Point2f> corners2[count];
  t[0] = getuTime();

  // cv::goodFeaturesToTrack(m[*reference], corners[*reference], 128,
  // 0.2, 10.0);

  // Features reference image
  cTFeatureSegmentContainer<float> seg1;
  seg1.segmentSetCount(SEG_WIDTH, SEG_HEIGHT);
  cTMostFeatureDetector<float, 3> fd1;
  fd1.setContainer(&seg1);
  fd1.detect(img[*reference].buffer, img[*reference].width,
             img[*reference].height, img[*reference].width, THRESHOLD);
  fd1.removeClose(MAX_DIST);

  for (int j = 0; j < seg1.segmentGetCount(); ++j) {
    if (seg1[j].status) {
      corners1[*reference].push_back(cv::Point2f(seg1[j].x, seg1[j].y));
    }
  }

  // Features overexposed image
  cTFeatureSegmentContainer<float> seg2;
  seg2.segmentSetCount(SEG_WIDTH, SEG_HEIGHT);
  cTMostFeatureDetector<float, 3> fd2;
  fd2.setContainer(&seg2);
  fd2.detect(img[over_exp].buffer, img[over_exp].width, img[over_exp].height,
             img[over_exp].width, THRESHOLD);
  fd2.removeClose(MAX_DIST);

  for (int j = 0; j < seg2.segmentGetCount(); ++j) {
    if (seg2[j].status) {
      corners2[over_exp].push_back(cv::Point2f(seg2[j].x, seg2[j].y));
    }
  }

  t[1] = getuTime();
  LOGI("Find features: %d us\n", t[1] - t[0]);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    histEqualization(gray[i].getData(),
                     gray[i].getWidth() * gray[i].getHeight());
  }
  t[1] = getuTime();
  LOGI("Histogram equalization: %d us\n", t[1] - t[0]);

  cv::Mat status1[count];
  cv::Mat err1[count];
  cv::Mat status2[count];
  cv::Mat err2[count];

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    if (i != *reference)
      cv::calcOpticalFlowPyrLK(m[*reference], m[i],  // src image,  dst image
                               corners1[*reference],
                               corners1[i],  // src points, dst points
                               status1[i], err1[i], cv::Size(21, 21), 3);

    if (i != over_exp)
      cv::calcOpticalFlowPyrLK(m[over_exp], m[i],  // src image,  dst image
                               corners2[over_exp],
                               corners2[i],  // src points, dst points
                               status2[i], err2[i], cv::Size(21, 21), 3);
  }
  t[1] = getuTime();
  LOGI("MOST: %d us\n", t[1] - t[0]);

  if (prog == 0) {
    t[0] = getuTime();

    for (int i = 0; i < count; ++i) {
      // Remove close points
      float minDistanceX = 0.03 * img[i].width;
      float minDistanceY = 0.03 * img[i].height;

      std::vector<cv::Point2f> sampl_corn1;
      std::vector<cv::Point2f> ref_corn1;
      std::vector<cv::Point2f> sampl_corn2;
      std::vector<cv::Point2f> ref_corn2;

      // Reference-Underexposed pair
      std::cout << "Reference-Underexposed pair" << std::endl;
      if (i != (*reference)) {
        int size = 0;
        std::cout << "Sample:      " << i << std::endl;
        std::cout << "Initial array size:  " << corners1[i].size() << std::endl;

        for (int p = 0; p < corners1[*reference].size(); ++p) {
          float dx = abs(corners1[i][p].x - corners1[*reference][p].x);
          float dy = abs(corners1[i][p].y - corners1[*reference][p].y);

          if ((dx <= minDistanceX) && (dy <= minDistanceY)) {
            sampl_corn1.push_back(corners1[i][p]);
            ref_corn1.push_back(corners1[*reference][p]);
            size++;
          }
        }
        std::cout << "New array size:  " << size << std::endl;
      } else {
        ref_corn1 = corners1[*reference];
        sampl_corn1 = corners1[*reference];
      }

      for (int p = 0; p < sampl_corn1.size(); ++p) {
        cv::circle(mO1[i], sampl_corn1[p], 5, cv::Scalar(0, 0, 0), 2);
        cv::circle(mO1[i], sampl_corn1[p], 2, cv::Scalar(255, 0, 0), 3);
      }

      if (i != *reference) {
        for (int p = 0; p < sampl_corn1.size(); ++p) {
          cv::line(mO1[i], sampl_corn1[p], ref_corn1[p], cv::Scalar(255, 0, 0),
                   2);
        }
      }

      // Overexposed-Reference pair
      std::cout << "\nOverexposed-Reference pair" << std::endl;
      if (i != over_exp) {
        int size = 0;
        std::cout << "Sample:      " << i << std::endl;
        std::cout << "Initial array size:  " << corners2[i].size() << std::endl;

        for (int p = 0; p < corners2[over_exp].size(); ++p) {
          float dx = abs(corners2[i][p].x - corners2[over_exp][p].x);
          float dy = abs(corners2[i][p].y - corners2[over_exp][p].y);

          if ((dx <= minDistanceX) && (dy <= minDistanceY)) {
            sampl_corn2.push_back(corners2[i][p]);
            ref_corn2.push_back(corners2[over_exp][p]);
            size++;
          }
        }
        std::cout << "New array size:  " << size << std::endl;
      } else {
        ref_corn2 = corners2[over_exp];
        sampl_corn2 = corners2[over_exp];
      }

      for (int p = 0; p < sampl_corn2.size(); ++p) {
        cv::circle(mO2[i], sampl_corn2[p], 5, cv::Scalar(0, 0, 0), 2);
        cv::circle(mO2[i], sampl_corn2[p], 2, cv::Scalar(255, 0, 0), 3);
      }

      if (i != over_exp) {
        for (int p = 0; p < sampl_corn2.size(); ++p) {
          cv::line(mO2[i], sampl_corn2[p], ref_corn2[p], cv::Scalar(255, 0, 0),
                   2);
        }
      }
    }

    for (int i = 0; i < count; ++i) {
      cv::namedWindow("pair1", 0);
      cv::namedWindow("pair2", 0);
      cv::imshow("pair1", mO1[i]);
      cv::imshow("pair2", mO2[i]);
      cv::waitKey(0);
    }

    t[1] = getuTime();
    LOGI("Draw: %d us\n", t[1] - t[0]);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  } else if (prog == 1 || prog == 2) {
    t[0] = getuTime();

    Test::GyroVStab::GlobalWarps<Test::Math::Vector<float, 2>, float> grid;
    Test::Math::Vector<float, 2>* temp_corner =
        (Test::Math::Vector<float, 2>*)(&corners[*reference][0].x);
    Test::Math::Vector<float, 2> mesh_grid[WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y];
    Test::Math::Vector<float, 2> reference_corner[corners[*reference].size()];
    for (int j = 0; j < corners[*reference].size(); j++) {
      reference_corner[j] = *(temp_corner + j);
      reference_corner[j].x() =
          2.0 * (reference_corner[j].x() / img[*reference].width) - 1.0;
      reference_corner[j].y() =
          2.0 * (reference_corner[j].y() / img[*reference].height) - 1.0;
    }

    grid.nowarp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);

    for (int i = 0; i < count; ++i) {
      if (i == *reference) continue;

      // Remove close points
      std::vector<cv::Point2f> sampl_corn;
      std::vector<cv::Point2f> ref_corn;
      float minDistanceX = 0.03 * img[i].width;
      float minDistanceY = 0.03 * img[i].height;

      int size = 0;
      std::cout << "Sample:      " << i << std::endl;
      std::cout << "Initial array size:  " << corners[i].size() << std::endl;

      for (int p = 0; p < corners[*reference].size(); ++p) {
        float dx = abs(corners[i][p].x - corners[*reference][p].x);
        float dy = abs(corners[i][p].y - corners[*reference][p].y);

        if ((dx <= minDistanceX) && (dy <= minDistanceY)) {
          sampl_corn.push_back(corners[i][p]);
          ref_corn.push_back(corners[*reference][p]);
          size++;
        }
      }
      std::cout << "New array size:  " << size << std::endl;

      Test::Math::Vector<float, 2>* temp_corner =
          (Test::Math::Vector<float, 2>*)(&corners[i][0].x);
      Test::Math::Vector<float, 2> sample_corner[sampl_corn.size()];
      for (int j = 0; j < sampl_corn.size(); j++) {
        sample_corner[j] = *(temp_corner + j);
        sample_corner[j].x() =
            2.0 * (sample_corner[j].x() / img[i].width) - 1.0;
        sample_corner[j].y() =
            2.0 * (sample_corner[j].y() / img[i].height) - 1.0;
      }

      grid.regPerspRobust(sample_corner, reference_corner, sampl_corn.size());

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
    LOGI("Regression + Warping: %d us\n", t[1] - t[0]);

    if (prog == 2) {
      unsigned short* img_sh[count];
      cTJpeg img_out;
      // img_out.create(mO[0].cols, mO[0].rows, 2);
      img_out.create(mO1[0].cols, mO1[0].rows, 2);
      // img_out.create(mO2[0].cols, mO2[0].rows, 2);

      // Convert the images to yuv422
      t[0] = getuTime();
      for (int i = 0; i < count; ++i) {
        // rgb24_to_yuv422(mO[i].data, mO[i].data, mO[i].cols, mO[i].rows,
        // mO[i].cols, mO[i].cols);
        rgb24_to_yuv422(mO1[i].data, mO1[i].data, mO1[i].cols, mO1[i].rows,
                        mO1[i].cols, mO1[i].cols);
        // rgb24_to_yuv422(mO2[i].data, mO2[i].data, mO2[i].cols, mO2[i].rows,
        // mO2[i].cols, mO2[i].cols);

        // img_sh[i] = (unsigned short*)mO[i].data;
        img_sh[i] = (unsigned short*)mO1[i].data;
        // img_sh[i] = (unsigned short*)mO2[i].data;
      }
      t[1] = getuTime();
      LOGI("RGB to YUV: %d us\n", t[1] - t[0]);

      // Fuse
      t[0] = getuTime();
      test_mfhdr_process(img_sh, count, (unsigned short*)img_out.getData(),
                         img_out.getWidth(), img_out.getHeight());
      t[1] = getuTime();
      LOGI("Fusion: %d us\n", t[1] - t[0]);

      // Convert back to RGB
      t[0] = getuTime();
      // yuv422_to_rgb24(img_out.getData(), mO[0].data, mO[0].cols, mO[0].rows,
      // mO[0].cols, mO[0].cols);
      yuv422_to_rgb24(img_out.getData(), mO1[0].data, mO1[0].cols, mO1[0].rows,
                      mO1[0].cols, mO1[0].cols);
      // yuv422_to_rgb24(img_out.getData(), mO2[0].data, mO2[0].cols,
      // mO2[0].rows, mO2[0].cols, mO2[0].cols);
      t[1] = getuTime();
      LOGI("YUV to RGB: %d us\n", t[1] - t[0]);
    }
  }
}
