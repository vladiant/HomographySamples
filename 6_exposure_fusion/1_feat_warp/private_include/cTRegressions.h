/*
 * File:   cTRegressions.h
 *
 * Created on February 8, 2012, 1:28 PM
 */

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

namespace reg_method {
enum reg_method { rotate, roll, affine };
}

cv::Mat reg_cv(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
               std::vector<unsigned char>& outliers,
               reg_method::reg_method method, short iterations);

// x =    x + by + c
// y = - bx +  y + f
bool reg_rotate(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
                std::vector<unsigned char>& outliers, double* coeff,
                short iterations);

bool reg_roll(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
              std::vector<unsigned char>& outliers, double* coeff,
              short iterations);

bool reg_affine(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
                std::vector<unsigned char>& outliers, double* coeff,
                short iterations);
