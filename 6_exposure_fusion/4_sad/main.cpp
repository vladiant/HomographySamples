/*
 * File:   main.cpp
 *
 * Created on June 12, 2012, 4:12 PM
 */

#include <cTFeatureSegmentContainer.h>
#include <cTJpeg.h>
#include <cTMostFeatureDetector.h>

#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "cTBruteSad.h"
#include "cTJpeg.h"
#include "imgconversion.h"

#define SEG_X_COUNT 8
#define SEG_Y_COUNT 6
#define MAX_DIST 0.3

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
  /*
  Mat img[2];
  img[0] = imread(argv[1], 0);
  img[1] = imread(argv[2], 0);
  */

  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " img1.jpg img2.jpg" << endl;
    return -1;
  }

  Mat img[2];
  cTJpeg jpeg[2];
  cTJpeg gray[2];

  jpeg[0].fileLoad(argv[1]);
  gray[0].create(jpeg[0].getWidth(), jpeg[0].getHeight(), 1);
  jpeg[0].fileSave("initial1.jpg", 100);
  rgb24_to_gray(jpeg[0].getData(), gray[0].getData(), jpeg[0].getWidth(),
                jpeg[0].getHeight(), jpeg[0].getWidth(), jpeg[0].getWidth());
  gray[0].fileSave("gray1.jpg", 100);
  img[0] = cv::Mat(gray[0].getHeight(), gray[0].getWidth(), CV_8UC1,
                   gray[0].getData());

  jpeg[1].fileLoad(argv[2]);
  gray[1].create(jpeg[1].getWidth(), jpeg[1].getHeight(), 1);
  jpeg[1].fileSave("initial2.jpg", 100);
  rgb24_to_gray(jpeg[1].getData(), gray[1].getData(), jpeg[1].getWidth(),
                jpeg[1].getHeight(), jpeg[1].getWidth(), jpeg[1].getWidth());
  gray[1].fileSave("gray2.jpg", 100);
  img[1] = cv::Mat(gray[1].getHeight(), gray[1].getWidth(), CV_8UC1,
                   gray[1].getData());

  cTFeatureSegmentContainer<float> feat[2];
  cTMostFeatureDetector<float, 1> most(&feat[0]);

  feat[0].segmentSetCount(SEG_X_COUNT, SEG_Y_COUNT);
  /*
  most.detect(
          img[0].data, img[0].cols, img[0].rows,
          img[0].cols, 20);
  */
  most.detect(img[0].data, img[0].cols, img[0].rows, img[0].cols, 20);
  most.removeCloseF(MAX_DIST * img[0].cols / SEG_X_COUNT);

  bruteForceSad(img[0].data, img[1].data, img[0].cols, img[1].rows, &feat[0],
                &feat[1], 16, 16, 128, 128);

  // feat[1] = feat[0];
  for (int i = 0; i < feat[0].segmentGetCount(); ++i) {
    if (feat[0][i].status == 0) continue;

    cv::line(img[0], Point(feat[0][i].x, feat[0][i].y),
             Point(feat[1][i].x, feat[1][i].y), 0, 3);
    cv::circle(img[0], Point(feat[0][i].x, feat[0][i].y), 6, 0, 3);
    cv::circle(img[0], Point(feat[0][i].x, feat[0][i].y), 3, 255, 3);
    cv::circle(img[0], Point(feat[1][i].x, feat[1][i].y), 6, 0, 3);
    cv::circle(img[0], Point(feat[1][i].x, feat[1][i].y), 3, 255, 3);

    cv::line(img[1], Point(feat[0][i].x, feat[0][i].y),
             Point(feat[1][i].x, feat[1][i].y), 0, 3);
    cv::circle(img[1], Point(feat[0][i].x, feat[0][i].y), 6, 0, 3);
    cv::circle(img[1], Point(feat[0][i].x, feat[0][i].y), 3, 255, 3);
    cv::circle(img[1], Point(feat[1][i].x, feat[1][i].y), 6, 0, 3);
    cv::circle(img[1], Point(feat[1][i].x, feat[1][i].y), 3, 255, 3);

    // std::cout << feat[0][i].x << " " << feat[0][i].y << " " << feat[1][i].x
    // << " " << feat[1][i].y << std::endl;
  }

  // for(int i = 1; i < 256; ++i) std::cout << int(exp(log(i)*2.2)+0.5) <<
  // std::endl;

  imwrite("d1.jpg", img[0]);
  imwrite("d2.jpg", img[1]);

  return 0;
}
