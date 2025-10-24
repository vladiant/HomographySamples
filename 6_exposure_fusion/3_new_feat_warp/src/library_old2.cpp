#include <math.h>
#include <stdio.h>
#include <string.h>

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
#include "library.h"

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
    float exposure[count];
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

  cv::Mat mO[count];
  for (int i = 0; i < count; ++i) {
    mO[i] = cv::Mat(img[i].height, img[i].width, CV_8UC3, img[i].buffer);
  }

  cv::Mat m[count];
  for (int i = 0; i < count; ++i) {
    m[i] = cv::Mat(gray[i].getHeight(), gray[i].getWidth(), CV_8UC1,
                   gray[i].getData());
  }

  cv::Mat corners[count];
  t[0] = getuTime();
  cv::goodFeaturesToTrack(m[*reference], corners[*reference], 128, 0.2, 10.0);
  t[1] = getuTime();
  LOGI("Find features: %d us\n", t[1] - t[0]);

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    histEqualization(gray[i].getData(),
                     gray[i].getWidth() * gray[i].getHeight());
  }
  t[1] = getuTime();
  LOGI("Histogram equalization: %d us\n", t[1] - t[0]);

  cv::Mat status[count];
  cv::Mat err[count];
  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    if (i == *reference) continue;
    cv::calcOpticalFlowPyrLK(m[*reference], m[i],  // src image,  dst image
                             corners[*reference],
                             corners[i],  // src points, dst points
                             status[i], err[i], cv::Size(21, 21), 3);
  }
  t[1] = getuTime();
  LOGI("KLT: %d us\n", t[1] - t[0]);

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
    LOGI("Draw: %d us\n", t[1] - t[0]);
  } else if (prog == 1 || prog == 2) {
    double coeff[count][9];
    std::vector<cv::Point2f> vcorners[count];
    std::vector<unsigned char> vstatus[count];
    for (int i = 0; i < count; ++i) {
      vcorners[i] = std::vector<cv::Point2f>(corners[i]);
      vstatus[i] = std::vector<unsigned char>(status[i]);
    }

    t[0] = getuTime();
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    Test::GyroVStab::GlobalWarps<Test::Math::Vector<float, 2>, float> grid;
    Test::Math::Vector<float, 2>* temp_corner =
        (Test::Math::Vector<float, 2>*)corners[*reference].data;
    Test::Math::Vector<float, 2> mesh_grid[WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y];
    Test::Math::Vector<float, 2> reference_corner[corners[*reference].rows];
    for (int j = 0; j < corners[*reference].rows; j++) {
      reference_corner[j] = *(temp_corner + j);
      reference_corner[j].x() =
          2.0 * (reference_corner[j].x() / img[*reference].width) - 1.0;
      reference_corner[j].y() =
          2.0 * (reference_corner[j].y() / img[*reference].height) - 1.0;
    }

    // std::cout << corners[*reference] << std::endl;

    grid.nowarp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);

    /*
    for( int j1=0; j1<WARP_GRID_SIZE_X*WARP_GRID_SIZE_Y; j1++)
            {
                    std::cout <<
    (1.0+mesh_grid[j1].x())*img[*reference].width*0.5 << "  " <<
    (1.0+mesh_grid[j1].y())*img[*reference].height*0.5 << std::endl;
                    //std::cout << mesh_grid[j1].x() << "  " <<
    mesh_grid[j1].y() << std::endl;
            }
    */

    for (int i = 0; i < count; ++i) {
      if (i == *reference) continue;

      Test::Math::Vector<float, 2>* temp_corner =
          (Test::Math::Vector<float, 2>*)corners[i].data;
      Test::Math::Vector<float, 2> sample_corner[corners[i].rows];
      for (int j = 0; j < corners[i].rows; j++) {
        sample_corner[j] = *(temp_corner + j);
        sample_corner[j].x() =
            2.0 * (sample_corner[j].x() / img[i].width) - 1.0;
        sample_corner[j].y() =
            2.0 * (sample_corner[j].y() / img[i].height) - 1.0;
      }

      grid.regPersp(sample_corner, reference_corner, corners[i].rows);

      grid.warp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);
      /*
      cv::Mat H = cv::Mat(3, 3, CV_64F);
      for (int j1=0; j1<3; j1++)
              for (int j2=0; j2<3; j2++) H.at<double>(j1,j2) =
      *(grid.getCoeffs()+3*j1+j2);

      H.at<double>(2,2) = 1;
      double vals1[] = { 2.0/img[i].width, 0.0, -1.0 , 0.0, 2.0/img[i].height,
      -1.0, 0.0, 0.0, 1.0 }; double vals2[] = { img[i].width/2.0, 0.0,
      img[i].width/2.0, 0.0, img[i].height/2.0, img[i].height/2.0, 0.0, 0.0, 1.0
      }; cv::Mat Transf(3, 3, CV_64F, vals1); cv::Mat TransfI(3, 3, CV_64F,
      vals2); cv::Mat H1=H*Transf; H=TransfI*H1; H/=H.at<double>(2,2);

      std::cout << H.inv() << std::endl;
      std::cout << H << std::endl;
      std::cout << corners[i] << std::endl;
      */
      cTJpeg temp;
      if (!temp.create(img[i].width, img[i].height, img[i].channels))
        std::cout << "@#%$!" << std::endl;

      cTImg tempimg = temp.getImg();
      for (int j = 0; j < img[i].width * img[i].height * img[i].channels; j++)
        *(tempimg.buffer + j) = 0;
      // for(int j=0; j<img[i].width*img[i].height*img[i].channels;j++)
      // *(tempimg.buffer+j) = *(img[i].buffer+j);

      // cTImg temp = img[i];
      // temp.width = img[i].width;
      // temp.height = img[i].height;
      // temp.channels = img[i].channels;
      // temp.buffer = img[i].buffer;

      // std::cout << *(&mesh_grid[0].x()+1) << std::endl;
      // std::cout << *(&mesh_grid[0].y()+1) << std::endl;
      // std::cout << sizeof(Test::Math::Vector<float,2>) << std::endl;
      // std::cout << sizeof(mesh_grid[0].y()) << std::endl;

      // std::cout << mesh_grid[0] << "  " << mesh_grid[1] << std::endl;
      // std::cout << mesh_grid[0].x() << "  " << mesh_grid[1].x() << std::endl;
      // std::cout << mesh_grid[0].y() << "  " << mesh_grid[1].y() << std::endl;
      // std::cout << *(&mesh_grid[0].x()+0) << "  " << *(&mesh_grid[0].x()+1)
      // << std::endl; std::cout << *(&mesh_grid[0].y()+0) << "  " <<
      // *(&mesh_grid[0].y()+1) << std::endl;

      Test::Math::Vector<float, 2>
          mesh_grid1[WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y];
      for (int j1 = 0; j1 < WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y; j1++) {
        float temp_x = mesh_grid[j1].x();
        float temp_y = mesh_grid[j1].y();
        mesh_grid1[j1].x() = (1.0 + temp_x) * img[i].width * 0.5;
        mesh_grid1[j1].y() = (1.0 + temp_y) * img[i].height * 0.5;
        // std::cout << mesh_grid1[j1].x() << "  " << mesh_grid1[j1].y() <<
        // std::endl;
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

      // temp.fileSave("test.jpg", 100, JCS_RGB);

      // cv::warpPerspective(mO[i].clone(), mO[i], H, m[i].size());

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    t[1] = getuTime();
    LOGI("Regression + Warping: %d us\n", t[1] - t[0]);

    if (prog == 2) {
      unsigned short* img_sh[count];
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
      LOGI("RGB to YUV: %d us\n", t[1] - t[0]);

      // Fuse
      t[0] = getuTime();
      test_mfhdr_process(img_sh, count, (unsigned short*)img_out.getData(),
                         img_out.getWidth(), img_out.getHeight());
      t[1] = getuTime();
      LOGI("Fusion: %d us\n", t[1] - t[0]);

      // Convert back to RGB
      t[0] = getuTime();
      yuv422_to_rgb24(img_out.getData(), mO[0].data, mO[0].cols, mO[0].rows,
                      mO[0].cols, mO[0].cols);
      t[1] = getuTime();
      LOGI("YUV to RGB: %d us\n", t[1] - t[0]);
    }
  }
}
