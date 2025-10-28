/*
 * File:   cTRegressions.cpp
 *
 * Created on February 8, 2012, 1:28 PM
 */

#include "cTRegressions.h"

cv::Mat reg_cv(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
               std::vector<unsigned char>& status,
               reg_method::reg_method method, short iterations) {
  cv::Mat H = cv::Mat(2, 3, CV_64FC1);
  double coeff[6];

  switch (method) {
    case reg_method::rotate:
      if (!reg_rotate(kp1, kp2, status, coeff, iterations))
        return cv::Mat::eye(3, 3, CV_64F);
      break;
    case reg_method::roll:
      if (!reg_roll(kp1, kp2, status, coeff, iterations))
        return cv::Mat::eye(3, 3, CV_64F);
      break;
    case reg_method::affine:
      if (!reg_affine(kp1, kp2, status, coeff, iterations))
        return cv::Mat::eye(3, 3, CV_64F);
      break;
    default:
      return cv::Mat::eye(3, 3, CV_64F);
  }

  H.at<double>(0, 0) = coeff[0];
  H.at<double>(0, 1) = coeff[1];
  H.at<double>(0, 2) = coeff[2];
  H.at<double>(1, 0) = coeff[3];
  H.at<double>(1, 1) = coeff[4];
  H.at<double>(1, 2) = coeff[5];
  // H.at<double>(2, 0) = 1.0;
  // H.at<double>(2, 1) = 1.0;
  // H.at<double>(2, 2) = 1.0;

  return H;
}

// x =    x + by + c
// y = - bx +  y + f
bool reg_rotate(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
                std::vector<unsigned char>& status, double* coeff,
                short iterations) {
  if (kp1.size() != kp2.size()) {
    std::cerr << "[Error] Affine regression keypoint size missmatch\n";
    return false;
  }

  double b, c, f;
  float x, y, xt, yt;
  float Sy2 = 0.0f, Sx2 = 0.0f, Sy = 0.0f, Sx = 0.0f, Sxt = 0.0f, Syt = 0.0f,
        Sxty = 0.0f, Sytx = 0.0f;
  float N = 0;
  float error = 0.0f, errc = 0.0f, std = 0.0f;
  // outliers.clear();	outliers.resize(kp1.size(), 0);

  for (int o = 0; o < kp1.size(); o++) {
    x = kp1[o].x;
    xt = kp2[o].x;
    y = kp1[o].y;
    yt = kp2[o].y;

    Sy += y;
    Sx += x;
    Sxt += xt;
    Syt += yt;
    Sy2 += y * y;
    Sx2 += x * x;
    Sxty += xt * y;
    Sytx += yt * x;

    ++N;
  }

  int it = 0;
  while (1) {
    float S1d[3][3] = {Sy2 + Sx2, Sy, -Sx, Sy, N, 0, -Sx, 0, N};
    float S2d[3] = {Sxty - Sytx, Sxt - Sx, Syt - Sy};

    cv::Mat S1 = cv::Mat(3, 3, CV_32FC1, S1d);
    cv::Mat S2 = cv::Mat(3, 1, CV_32FC1, S2d);

    cv::Mat S = S1.inv() * S2;

    b = S.at<float>(0, 0);
    c = S.at<float>(1, 0);
    f = S.at<float>(2, 0);

    std::vector<float> err;
    err.resize(kp1.size(), 0.0f);
    error = 0.0f;
    float oldstd = std;
    std = 0.0f;
    for (int o = 0; o < kp1.size(); o++) {
      errc = (kp1[o].x * 1 + kp1[o].y * b + c - kp2[o].x) *
                 (kp1[o].x * 1 + kp1[o].y * b + c - kp2[o].x) +
             (-kp1[o].x * b + kp1[o].y * 1 + f - kp2[o].y) *
                 (-kp1[o].x * b + kp1[o].y * 1 + f - kp2[o].y);

      errc *= status[o];
      error += errc;
      err[o] = errc;
    }
    float merr = error / err.size();
    for (int o = 0; o < err.size(); o++) {
      std += (err[o] - merr) * (err[o] - merr);
    }

    std = sqrt(std / (N - 1));
    if (oldstd == std) break;

    // ****************************** REMOVE OUTLIERS **********************
    for (int o = 0; o < err.size(); o++) {
      if (err[o] > 3 * std && status[o] == 1) {
        status[o] = 0;

        x = kp1[o].x;
        xt = kp2[o].x;
        y = kp1[o].y;
        yt = kp2[o].y;

        Sy -= y;
        Sx -= x;
        Sxt -= xt;
        Syt -= yt;
        Sy2 -= y * y;
        Sx2 -= x * x;
        Sxty -= xt * y;
        Sytx -= yt * x;

        --N;
      }
    }

    if (++it > iterations) break;
  }

  coeff[0] = 1.0f;
  coeff[1] = b;
  coeff[2] = c;
  coeff[3] = -b;
  coeff[4] = 1.0f;
  coeff[5] = f;

  return true;
}

// **********************************  x' =  x + by + c		Regresssion
// **********************************  y' = dx + ey + f		Min Err
bool reg_roll(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
              std::vector<unsigned char>& status, double* coeff,
              short iterations) {
  if (kp1.size() != kp2.size()) {
    std::cerr << "[Error] Affine regression keypoint size missmatch\n";
    return false;
  }

  double b, c, d, e, f;
  float x, y, xt, yt;
  double Sy2 = 0.0f, Sy = 0.0f, N = 0.0f, Smxy = 0.0f, Smx = 0.0f;
  double Syt = 0.0f, Syty = 0.0f, Sxy = 0.0f, Sytx = 0.0f, Sx = 0.0f,
         Sx2 = 0.0f;
  float error = 0.0f, errc = 0.0f, std = 0.0f;

  for (int o = 0; o < kp1.size(); o++) {
    x = kp1[o].x;
    xt = kp2[o].x;
    y = kp1[o].y;
    yt = kp2[o].y;

    Sy2 += y * y;
    Sy += y;
    Smxy += (x - xt) * y;
    Smx += (x - xt);

    Syt += yt;
    Syty += yt * y;
    Sxy += x * y;
    Sytx += yt * x;
    Sx += x;
    Sx2 += x * x;

    N++;
  }

  int it = 0;
  while (1) {
    if ((Sy * Sy - N * Sy2) == 0 || Sy == 0) {
      std::cout << "*************** Division by ZERO! ***************\n";
      return false;
    }
    b = (N * Smxy - Smx * Sy) / (Sy * Sy - N * Sy2);
    c = (-Smxy - b * Sy2) / Sy;

    /*cv::Mat S1 = cv::Mat(3, 1, CV_64FC1);
    cv::Mat S2 = cv::Mat(3, 3, CV_64FC1);
    S2.at<double>(0, 0) = Sx2;
    S2.at<double>(0, 1) = Sxy;
    S2.at<double>(0, 2) = Sx;
    S2.at<double>(1, 0) = Sxy;
    S2.at<double>(1, 1) = Sy2;
    S2.at<double>(1, 2) = Sy;
    S2.at<double>(2, 0) = Sx;
    S2.at<double>(2, 1) = Sy;
    S2.at<double>(2, 2) = N;
    cv::Mat S3 = cv::Mat(3, 1, CV_64FC1);
    S3.at<double>(0, 0) = Sytx;
    S3.at<double>(1, 0) = Syty;
    S3.at<double>(2, 0) = Syt;

    S1 = S2.inv() * S3;*/

    d = (Sxy * (Syty * N - Sy * Syt) + Sytx * (Sy * Sy - Sy2 * N) +
         Sx * (Sy2 * Syt - Sy * Syty)) /
        (Sx2 * (Sy * Sy - Sy2 * N) + Sxy * Sxy * N - 2 * Sx * Sy * Sxy +
         Sx * Sx * Sy2);
    e = (-N * d * Sxy + Syty * N - Sy * Syt + d * Sy * Sx) /
        (Sy2 * N - Sy * Sy);
    f = (Syt - d * Sx - e * Sy) / N;

    std::vector<float> err;
    err.resize(kp1.size(), 0.0f);
    error = 0.0f;
    float oldstd = std;
    std = 0.0f;
    for (int o = 0; o < kp1.size(); o++) {
      errc = (kp1[o].x * 1 + kp1[o].y * b + c - kp2[o].x) *
                 (kp1[o].x * 1 + kp1[o].y * b + c - kp2[o].x) +
             (kp1[o].x * d + kp1[o].y * e + f - kp2[o].y) *
                 (kp1[o].x * d + kp1[o].y * e + f - kp2[o].y);

      errc *= status[o];
      error += errc;
      err[o] = errc;
    }
    float merr = error / err.size();
    for (int o = 0; o < err.size(); o++) {
      std += (err[o] - merr) * (err[o] - merr);
    }

    std = sqrt(std / (N - 1));
    if (oldstd == std) break;

    // ****************************** REMOVE OUTLIERS **********************
    for (int o = 0; o < err.size(); o++) {
      if (err[o] > 3 * std && status[o] == 1) {
        status[o] = 0;

        x = kp1[o].x;
        xt = kp2[o].x;
        y = kp1[o].y;
        yt = kp2[o].y;

        Sy2 -= y * y;
        Sy -= y;
        Smxy -= (x - xt) * y;
        Smx -= (x - xt);

        Syt -= yt;
        Syty -= yt * y;
        Sxy -= x * y;
        Sytx -= yt * x;
        Sx -= x;
        Sx2 -= x * x;

        --N;
      }
    }

    if (++it > iterations) break;
  }

  coeff[0] = 1.0f;
  coeff[1] = b;
  coeff[2] = c;
  coeff[3] = d;
  coeff[4] = e;
  coeff[5] = f;

  return true;
}

// **********************************  x' = ax + by + c		Regresssion
// **********************************  y' = dx + ey + f		Min Err
bool reg_affine(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
                std::vector<unsigned char>& status, double* coeff,
                short iterations) {
  if (kp1.size() != kp2.size()) {
    std::cerr << "[Error] Affine regression keypoint size missmatch\n";
    return false;
  }

  double a, b, c, d, e, f;
  float x, y, xt, yt;
  double Sx2 = 0.0f, Sy2 = 0.0f, Sxy = 0.0f, Sx = 0.0f, Sy = 0.0f, Sxtx = 0.0f,
         Sxty = 0.0f, Sxt = 0.0f, Sytx = 0.0f, Syty = 0.0f, Syt = 0.0f;
  double N = 0;
  double error = 0.0f, errc = 0.0f, std = 0.0f, merr = 0.0;

  for (int o = 0; o < kp1.size(); o++) {
    x = kp1[o].x;
    xt = kp2[o].x;
    y = kp1[o].y;
    yt = kp2[o].y;

    Sx2 += x * x;
    Sy2 += y * y;
    Sxy += x * y;
    Sx += x;
    Sy += y;
    Sxtx += xt * x;
    Sxty += xt * y;
    Sxt += xt;
    Sytx += yt * x;
    Syty += yt * y;
    Syt += yt;
    ++N;
  }

  int it = 0;
  while (1) {
    double S1d[3][3] = {Sx2, Sxy, Sx, Sxy, Sy2, Sy, Sx, Sy, N};
    double S2d[3] = {Sxtx, Sxty, Sxt};

    cv::Mat S1 = cv::Mat(3, 3, CV_64FC1, S1d);
    cv::Mat S2 = cv::Mat(3, 1, CV_64FC1, S2d);

    cv::Mat S = S1.inv() * S2;

    a = S.at<double>(0, 0);
    b = S.at<double>(1, 0);
    c = S.at<double>(2, 0);

    double S2d2[3] = {Sytx, Syty, Syt};
    S2 = cv::Mat(3, 1, CV_64FC1, S2d2);

    S = S1.inv() * S2;

    d = S.at<double>(0, 0);
    e = S.at<double>(1, 0);
    f = S.at<double>(2, 0);

    std::vector<double> err;
    err.resize(kp1.size(), 0.0f);
    error = 0.0f;
    // double oldstd = std;
    double oldmerr = merr;
    std = 0.0f;
    int errcnt = 0;
    for (int o = 0; o < kp1.size(); o++) {
      errc = (kp1[o].x * a + kp1[o].y * b + c - kp2[o].x) *
                 (kp1[o].x * a + kp1[o].y * b + c - kp2[o].x) +
             (kp1[o].x * d + kp1[o].y * e + f - kp2[o].y) *
                 (kp1[o].x * d + kp1[o].y * e + f - kp2[o].y);

      errc *= status[o];
      errcnt += status[o];
      error += errc;
      err[o] = errc;
    }
    merr = error / errcnt;
    if (oldmerr == merr) break;

    /*for(int o = 0; o < err.size(); o++)
    {
            std += (err[o] - merr)*(err[o] - merr);
    }

    std = sqrt(std / (N - 1));
    if(oldstd == std) break;*/

    // ****************************** REMOVE OUTLIERS **********************
    for (int o = 0; o < err.size(); o++) {
      if (err[o] > 9 * merr && status[o] == 1)
      // if(err[o] > 3*std && status[o] == 1)
      {
        status[o] = 0;

        x = kp1[o].x;
        xt = kp2[o].x;
        y = kp1[o].y;
        yt = kp2[o].y;

        Sx2 -= x * x;
        Sy2 -= y * y;
        Sxy -= x * y;
        Sx -= x;
        Sy -= y;
        Sxtx -= xt * x;
        Sxty -= xt * y;
        Sxt -= xt;
        Sytx -= yt * x;
        Syty -= yt * y;
        Syt -= yt;

        --N;
      }
    }

    if (++it > iterations) break;
  }

  coeff[0] = a;
  coeff[1] = b;
  coeff[2] = c;
  coeff[3] = d;
  coeff[4] = e;
  coeff[5] = f;

  return true;
}

/*bool reg_affine(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f>& kp2,
std::vector<unsigned char>& status, float* coeff)
{
        if(kp1.size() != kp2.size())
        {
                std::cerr<<"[Error] Affine regression keypoint size
missmatch\n"; return false;
        }

        float a, b, c, d, e, f;
        float x, y, xt, yt;
        float Sx2 = 0.0f, Sy2 = 0.0f, Sxy = 0.0f, Sx = 0.0f, Sy = 0.0f, Sxtx =
0.0f, Sxty = 0.0f, Sxt = 0.0f, Sytx = 0.0f, Syty = 0.0f, Syt = 0.0f; int N = 0;
        float error = 0.0f, errc = 0.0f, std = 0.0f;

        for(int o = 0; o < kp1.size(); o++)
        {
                x = kp1[o].x;		xt = kp2[o].x;
                y = kp1[o].y;		yt = kp2[o].y;

                Sx2		+= x*x;
                Sy2		+= y*y;
                Sxy		+= x*y;
                Sx		+= x;
                Sy		+= y;
                Sxtx	+= xt*x;
                Sxty	+= xt*y;
                Sxt		+= xt;
                Sytx	+= yt*x;
                Syty	+= yt*y;
                Syt		+= yt;
                ++N;
        }

        int it = 0;
        while(1)
        {
                float S1d[3][3] = {Sx2, Sxy, Sx, Sxy, Sy2, Sy, Sx, Sy, N};
                float S2d[3] = {Sxtx, Sxty, Sxt};

                cv::Mat S1 = cv::Mat(3, 3, CV_32FC1, S1d);
                cv::Mat S2 = cv::Mat(3, 1, CV_32FC1, S2d);

                cv::Mat S = S1.inv() * S2;

                a = S.at<float>(0, 0);
                b = S.at<float>(1, 0);
                c = S.at<float>(2, 0);

                float S2d2[3] = {Sytx, Syty, Syt};
                S2 = cv::Mat(3, 1, CV_32FC1, S2d2);

                S = S1.inv() * S2;

                d = S.at<float>(0, 0);
                e = S.at<float>(1, 0);
                f = S.at<float>(2, 0);

                std::vector<float> err;	err.resize(kp1.size(), 0.0f);
                error = 0.0f;
                float oldstd = std;
                std = 0.0f;
                for(int o = 0; o < kp1.size(); o++)
                {
                        errc =	(kp1[o].x*a + kp1[o].y*b + c - kp2[o].x)*
                                        (kp1[o].x*a + kp1[o].y*b + c - kp2[o].x)
+ (kp1[o].x*d + kp1[o].y*e + f - kp2[o].y)* (kp1[o].x*d + kp1[o].y*e + f -
kp2[o].y);

                        errc *= status[o];
                        error += errc;
                        err[o] = errc;
                }
                float merr = error / err.size();
                for(int o = 0; o < err.size(); o++)
                {
                        std += (err[o] - merr)*(err[o] - merr);
                }

                std = sqrt(std / (N - 1));
                if(oldstd == std) break;

                // ****************************** REMOVE OUTLIERS
********************** for(int o = 0; o < err.size(); o++)
                {
                        if(err[o] > 3*std && status[o] == 1)
                        {
                                status[o] = 0;

                                x = kp1[o].x;		xt = kp2[o].x;
                                y = kp1[o].y;		yt = kp2[o].y;

                                Sx2		-= x*x;
                                Sy2		-= y*y;
                                Sxy		-= x*y;
                                Sx		-= x;
                                Sy		-= y;
                                Sxtx	-= xt*x;
                                Sxty	-= xt*y;
                                Sxt		-= xt;
                                Sytx	-= yt*x;
                                Syty	-= yt*y;
                                Syt		-= yt;

                                --N;
                        }
                }

                if(++it > 20)
                        break;
        }

        coeff[0] = a;
        coeff[1] = b;
        coeff[2] = c;
        coeff[3] = d;
        coeff[4] = e;
        coeff[5] = f;

        return true;
}
*/
