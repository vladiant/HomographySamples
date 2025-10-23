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
#include "gamma.h"
#include "imgconversion.h"
#include "library.h"

#define MAX_IN_OUT_FILES 16

#define WARP_GRID_SIZE_X 16
#define WARP_GRID_SIZE_Y 12
#define SEG_X_COUNT 16
#define SEG_Y_COUNT 12
#define THRESHOLD 20
#define MAX_DIST 0.3
#define MAX_MOTION_VECTOR 0.03

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

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    gray[i].create(img[i].width, img[i].height, 1);
    rgb24_to_gray_gamma_reverse(img[i].buffer, gray[i].getData(), img[i].width,
                                img[i].height, img[i].width, img[i].width);
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

    // To be read from file
    exposure[0] = 1;
    exposure[1] = 1;
    exposure[2] = 20;

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

  std::vector<cv::Point2f> corners[count];
  std::vector<cv::Point2f> refcorners[count];
  t[0] = getuTime();

  for (int i = 0; i < count; ++i) {
    cTJpeg tempgray;
    tempgray.create(320, 240, 1);
    cv::Mat mtemp = cv::Mat(gray[i].getHeight(), gray[i].getWidth(), CV_8UC1,
                            gray[i].getData());
    cv::Mat mvga = cv::Mat(480, 640, CV_8UC1);
    cv::resize(mtemp, mvga, mvga.size(), 0, 0, cv::INTER_NEAREST);
    cv::Mat mqvga = cv::Mat(240, 320, CV_8UC1, tempgray.getData());
    cv::resize(mvga, mqvga, mqvga.size(), 0, 0, cv::INTER_LINEAR);

    cTFeatureSegmentContainer<float> seg;
    seg.segmentSetCount(SEG_X_COUNT, SEG_Y_COUNT);
    cTMostFeatureDetector<float, 1> fd;
    fd.setContainer(&seg);

    // search features in QVGA, match them in full resolution
    fd.detect(tempgray.getData(), tempgray.getWidth(), tempgray.getHeight(),
              tempgray.getWidth(), THRESHOLD);
    fd.removeClose(MAX_DIST * tempgray.getWidth() / SEG_X_COUNT);

    for (int j = 0; j < seg.segmentGetCount(); ++j) {
      if (seg[j].status) {
        // feature coordinates in full resolution
        corners[i].push_back(cv::Point2f(
            seg[j].x * gray[i].getWidth() / float(tempgray.getWidth()),
            seg[j].y * gray[i].getHeight() / float(tempgray.getHeight())));
      }
    }
  }

  refcorners[*reference] = corners[*reference];

  t[1] = getuTime();
  LOGI("Find features: %d us\n", t[1] - t[0]);

  cv::Mat status[count];
  cv::Mat err[count];

  t[0] = getuTime();
  for (int i = 0; i < count; ++i) {
    if (i == *reference) continue;
    if (exposure[i] > exposure[*reference]) {
      float exp_ratio = exposure[i] / exposure[*reference];
      std::cout << i << "  " << exp_ratio << std::endl;

      cTJpeg graytemp;
      graytemp.create(gray[*reference].getWidth(), gray[*reference].getHeight(),
                      1);

      cv::Mat mtemp =
          cv::Mat(gray[*reference].getHeight(), gray[*reference].getWidth(),
                  CV_8UC1, graytemp.getData());

      for (int j = 0;
           j < gray[*reference].getWidth() * gray[*reference].getHeight();
           j++) {
        float temp = *(gray[*reference].getData() + j);
        temp *= exp_ratio;
        temp = temp > 255 ? 255 : temp;
        *(graytemp.getData() + j) = uchar(temp);
      }
      /*
      cv::namedWindow("reference", 0);
      cv::namedWindow("overexp", 0);
      cv::imshow("reference", mtemp);
      cv::imshow("overexp", m[i]);
      cv::waitKey(0);
      cv::destroyWindow("reference");
      cv::destroyWindow("overexp");

      char buf[] = "exp_00.jpg";
      buf[4] = '0'+i;
      buf[5] = '0'+(*reference);
      cv::imwrite(buf,m[i]);
      buf[3] = 'u';
      cv::imwrite(buf,mtemp);
      */
      cv::calcOpticalFlowPyrLK(m[i], mtemp,  // src image,  dst image
                               corners[i],
                               refcorners[i],  // src points, dst points
                               status[i], err[i], cv::Size(21, 21), 3);
    } else {
      float exp_ratio = exposure[*reference] / exposure[i];
      std::cout << i << "  " << exp_ratio << std::endl;

      cTJpeg graytemp;
      graytemp.create(gray[i].getWidth(), gray[i].getHeight(), 1);

      cv::Mat mtemp = cv::Mat(gray[i].getHeight(), gray[i].getWidth(), CV_8UC1,
                              graytemp.getData());

      for (int j = 0; j < gray[i].getWidth() * gray[i].getHeight(); j++) {
        float temp = *(gray[i].getData() + j);
        temp *= exp_ratio;
        temp = temp > 255 ? 255 : temp;
        *(graytemp.getData() + j) = uchar(temp);
      }
      /*
      cv::namedWindow("reference", 0);
      cv::namedWindow("underexp", 0);
      cv::imshow("reference", m[*reference]);
      cv::imshow("underexp", mtemp);
      cv::waitKey(0);
      cv::destroyWindow("reference");
      cv::destroyWindow("underexp");

      char buf[] = "exp_00.jpg";
      buf[4] = '0'+(*reference);
      buf[5] = '0'+i;
      cv::imwrite(buf,m[*reference]);
      buf[3] = 'o';
      cv::imwrite(buf,mtemp);
      */
      cv::calcOpticalFlowPyrLK(m[*reference], mtemp,  // src image,  dst image
                               corners[*reference],
                               corners[i],  // src points, dst points
                               status[i], err[i], cv::Size(21, 21), 3);

      refcorners[i] = corners[*reference];
    }
  }
  t[1] = getuTime();
  LOGI("KLT: %d us\n", t[1] - t[0]);

  if (prog == 0) {
    t[0] = getuTime();

    for (int i = 0; i < count; ++i) {
      // Remove close points
      std::vector<cv::Point2f> sampl_corn;
      std::vector<cv::Point2f> ref_corn;
      float minDistanceX = MAX_MOTION_VECTOR * gray[i].getWidth();
      float minDistanceY = MAX_MOTION_VECTOR * gray[i].getHeight();

      if (i != (*reference)) {
        int size = 0;

        for (int p = 0; p < refcorners[i].size(); ++p) {
          float dx = abs(corners[i][p].x - refcorners[i][p].x);
          float dy = abs(corners[i][p].y - refcorners[i][p].y);

          if ((dx <= minDistanceX) && (dy <= minDistanceY)) {
            sampl_corn.push_back(corners[i][p]);
            ref_corn.push_back(refcorners[i][p]);
            size++;
          }
        }
      } else {
        ref_corn = corners[*reference];
        sampl_corn = corners[*reference];
      }

      if (i != *reference) {
        for (int p = 0; p < sampl_corn.size(); ++p) {
          cv::Point2f temp_corn = sampl_corn[p];

          temp_corn.x *= img[i].width / float(gray[i].getWidth());
          temp_corn.y *= img[i].height / float(gray[i].getHeight());

          cv::circle(mO[i], temp_corn, 5, cv::Scalar(0, 0, 0), 2);
          cv::circle(mO[i], temp_corn, 2, cv::Scalar(255, 0, 0), 3);
        }

        for (int p = 0; p < sampl_corn.size(); ++p) {
          cv::Point2f temp_corn = sampl_corn[p];

          temp_corn.x *= img[i].width / float(gray[i].getWidth());
          temp_corn.y *= img[i].height / float(gray[i].getHeight());

          cv::Point2f temp_corn1 = ref_corn[p];

          temp_corn1.x *= img[i].width / float(gray[i].getWidth());
          temp_corn1.y *= img[i].height / float(gray[i].getHeight());

          cv::line(mO[i], temp_corn, temp_corn1, cv::Scalar(255, 0, 0), 2);
        }
      }

      // Draws corresponding feature for each sample
      const int point_color = 0xFF0000 >> ((i) * 24 / count);
      unsigned int point_colorR = ((0xFF0000) & (point_color)) >> 16;
      unsigned int point_colorG = ((0x00FF00) & (point_color)) >> 8;
      unsigned int point_colorB = (0x0000FF) & (point_color);

      for (int p = 0; p < sampl_corn.size(); ++p) {
        cv::Point2f temp_corn1 = ref_corn[p];

        temp_corn1.x *= img[i].width / float(gray[i].getWidth());
        temp_corn1.y *= img[i].height / float(gray[i].getHeight());

        cv::circle(mO[*reference], temp_corn1, 5, cv::Scalar(0, 0, 0), 2);
        cv::circle(mO[*reference], temp_corn1, 2,
                   cv::Scalar(point_colorR, point_colorG, point_colorB), 3);
      }
    }

    t[1] = getuTime();
    LOGI("Draw: %d us\n", t[1] - t[0]);
  } else if (prog == 1 || prog == 2) {
    t[0] = getuTime();

    Test::GyroVStab::GlobalWarps<Test::Math::Vector<float, 2>, float> grid;
    Test::Math::Vector<float, 2> mesh_grid[WARP_GRID_SIZE_X * WARP_GRID_SIZE_Y];

    grid.nowarp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);

    for (int i = 0; i < count; ++i) {
      if (i == *reference) continue;

      // Remove close points
      std::vector<cv::Point2f> sampl_corn;
      std::vector<cv::Point2f> ref_corn;
      float minDistanceX = MAX_MOTION_VECTOR * gray[i].getWidth();
      float minDistanceY = MAX_MOTION_VECTOR * gray[i].getHeight();

      int size = 0;

      for (int p = 0; p < corners[i].size(); ++p) {
        float dx = abs(corners[i][p].x - refcorners[i][p].x);
        float dy = abs(corners[i][p].y - refcorners[i][p].y);

        if ((dx <= minDistanceX) && (dy <= minDistanceY)) {
          sampl_corn.push_back(corners[i][p]);
          ref_corn.push_back(refcorners[i][p]);
          size++;
        }
      }

      Test::Math::Vector<float, 2>* temp_corner =
          (Test::Math::Vector<float, 2>*)(&ref_corn[0].x);
      Test::Math::Vector<float, 2> reference_corner[ref_corn.size()];
      for (int j = 0; j < ref_corn.size(); j++) {
        reference_corner[j] = *(temp_corner + j);
        reference_corner[j].x() =
            2.0 * (reference_corner[j].x() / gray[*reference].getWidth()) - 1.0;
        reference_corner[j].y() =
            2.0 * (reference_corner[j].y() / gray[*reference].getHeight()) -
            1.0;
      }

      temp_corner = (Test::Math::Vector<float, 2>*)(&sampl_corn[0].x);
      Test::Math::Vector<float, 2> sample_corner[sampl_corn.size()];
      for (int j = 0; j < sampl_corn.size(); j++) {
        sample_corner[j] = *(temp_corner + j);
        sample_corner[j].x() =
            2.0 * (sample_corner[j].x() / gray[i].getWidth()) - 1.0;
        sample_corner[j].y() =
            2.0 * (sample_corner[j].y() / gray[i].getHeight()) - 1.0;
      }

      grid.regPerspRobust(sample_corner, reference_corner, sampl_corn.size());

      grid.warp(mesh_grid, WARP_GRID_SIZE_X, WARP_GRID_SIZE_Y);

      /*
      cv::Mat H = cv::Mat(3, 3, CV_64F);
      for (int j1=0; j1<3; j1++)
              for (int j2=0; j2<3; j2++) H.at<double>(j1,j2) =
      *(grid.getCoeffs()+3*j1+j2); H.at<double>(2,2) = 1;

      //double vals1[] = { 2.0/img[i].width, 0.0, -1.0 , 0.0, 2.0/img[i].height,
      -1.0, 0.0, 0.0, 1.0 };
      //double vals2[] = { img[i].width/2.0, 0.0, img[i].width/2.0, 0.0,
      img[i].height/2.0, img[i].height/2.0, 0.0, 0.0, 1.0 };
      //QVGA coords
      double vals1[] = { 2.0/320, 0.0, -1.0 , 0.0, 2.0/240, -1.0, 0.0, 0.0, 1.0
      }; double vals2[] = { 320/2.0, 0.0, 320/2.0, 0.0, 240/2.0, 240/2.0, 0.0,
      0.0, 1.0 }; cv::Mat Transf(3, 3, CV_64F, vals1); cv::Mat TransfI(3, 3,
      CV_64F, vals2); cv::Mat H1=H*Transf; H=TransfI*H1; H/=H.at<double>(2,2);

      std::cout << H.inv() << std::endl;
      std::cout << H << std::endl;
      */

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
