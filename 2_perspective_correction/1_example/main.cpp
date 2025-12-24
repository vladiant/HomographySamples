#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

namespace {

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

void calcChessboardCorners(Size boardSize, float squareSize,
                           vector<Point3f> &corners,
                           Pattern patternType = CHESSBOARD) {
  corners.resize(0);
  switch (patternType) {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
              Point3f(float(j * squareSize), float(i * squareSize), 0));
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
                                    float(i * squareSize), 0));
      break;
    default:
      CV_Error(Error::StsBadArg, "Unknown pattern type\n");
  }
}

Mat computeHomography(const Mat &R_1to2, const Mat &tvec_1to2,
                      const double d_inv, const Mat &normal) {
  Mat homography = R_1to2 + d_inv * tvec_1to2 * normal.t();
  return homography;
}

void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2,
                  const Mat &tvec2, Mat &R_1to2, Mat &tvec_1to2) {
  // c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
  R_1to2 = R2 * R1.t();
  tvec_1to2 = R2 * (-R1.t() * tvec1) + tvec2;
}

void decomposeHomography() {
  const vector<Point2f> corners1{
      {256.641, 357.179}, {255.054, 334.421}, {254.73, 308.614},
      {253.456, 280.523}, {252.559, 248.439}, {252.434, 213.824},
      {251.238, 172.526}, {251.795, 128.655}, {251.45, 77.9899},
      {291.726, 366.075}, {292.574, 343.537}, {293.29, 318.045},
      {294.874, 290.144}, {296.781, 257.949}, {298.889, 221.936},
      {301.447, 182.748}, {304.364, 135.672}, {306.864, 86.4006},
      {327.711, 374.479}, {331.258, 352.658}, {334.2, 327.505},
      {337.75, 299.083},  {342.469, 267.664}, {347.641, 233.14},
      {352.462, 192.016}, {357.415, 145.87},  {365.41, 96.5842},
      {367, 382},         {369.487, 360.44},  {375.125, 335.709},
      {381.419, 308.436}, {388.777, 277.794}, {396.53, 241.762},
      {404.763, 202.709}, {413.401, 158.33},  {423.243, 106.813},
      {401.614, 389.562}, {407.986, 368.555}, {416.625, 344.214},
      {425.324, 317.515}, {433.687, 287.025}, {445.412, 252.542},
      {456.43, 213.699},  {469.124, 169.155}, {482.926, 120.31},
      {437.728, 396.497}, {446.419, 376.198}, {456.982, 352.071},
      {467.085, 326.68},  {480.337, 296.202}, {492.632, 262.762},
      {508.201, 224.353}, {523.361, 181.191}, {539.248, 131.8},
  };

  const vector<Point2f> corners2{
      {244.454, 94.3314}, {274.622, 92.2413}, {305.494, 90.4029},
      {338.364, 88.8363}, {371.592, 87.9836}, {406.844, 86.9164},
      {441.633, 86.3721}, {477.623, 86.3797}, {513.987, 86.7991},
      {245, 126.538},     {274.968, 124.752}, {306.044, 124.032},
      {338.715, 123.265}, {372.094, 122.129}, {406.648, 122.168},
      {442.194, 122.246}, {477.867, 122.105}, {514.033, 122.921},
      {245.47, 158.355},  {275.371, 158.377}, {306.421, 157.62},
      {338.708, 157.422}, {372.355, 157.376}, {406.544, 157.492},
      {442.119, 157.677}, {477.643, 158.417}, {513.531, 159.281},
      {246.383, 190.263}, {275.796, 190.639}, {307.274, 191.097},
      {339.184, 191.576}, {372.374, 191.871}, {406.952, 192.447},
      {441.551, 193.68},  {477.331, 194.333}, {513.256, 195.527},
      {247.433, 222.467}, {276.823, 223.436}, {307.433, 224.244},
      {339.558, 225.207}, {372.943, 226.424}, {406.426, 227.532},
      {441.408, 228.535}, {476.718, 230.1},   {511.529, 231.599},
      {248.8, 253.599},   {277.509, 255.177}, {308.577, 256.44},
      {340.387, 258.295}, {372.752, 259.799}, {406.105, 261.659},
      {440.436, 263.306}, {475.415, 264.713}, {510.177, 266.199},
  };

  const vector<Point3f> objectPoints{
      {0, 0, 0},         {0.025, 0, 0},    {0.05, 0, 0},      {0.075, 0, 0},
      {0.1, 0, 0},       {0.125, 0, 0},    {0.15, 0, 0},      {0.175, 0, 0},
      {0.2, 0, 0},       {0, 0.025, 0},    {0.025, 0.025, 0}, {0.05, 0.025, 0},
      {0.075, 0.025, 0}, {0.1, 0.025, 0},  {0.125, 0.025, 0}, {0.15, 0.025, 0},
      {0.175, 0.025, 0}, {0.2, 0.025, 0},  {0, 0.05, 0},      {0.025, 0.05, 0},
      {0.05, 0.05, 0},   {0.075, 0.05, 0}, {0.1, 0.05, 0},    {0.125, 0.05, 0},
      {0.15, 0.05, 0},   {0.175, 0.05, 0}, {0.2, 0.05, 0},    {0, 0.075, 0},
      {0.025, 0.075, 0}, {0.05, 0.075, 0}, {0.075, 0.075, 0}, {0.1, 0.075, 0},
      {0.125, 0.075, 0}, {0.15, 0.075, 0}, {0.175, 0.075, 0}, {0.2, 0.075, 0},
      {0, 0.1, 0},       {0.025, 0.1, 0},  {0.05, 0.1, 0},    {0.075, 0.1, 0},
      {0.1, 0.1, 0},     {0.125, 0.1, 0},  {0.15, 0.1, 0},    {0.175, 0.1, 0},
      {0.2, 0.1, 0},     {0, 0.125, 0},    {0.025, 0.125, 0}, {0.05, 0.125, 0},
      {0.075, 0.125, 0}, {0.1, 0.125, 0},  {0.125, 0.125, 0}, {0.15, 0.125, 0},
      {0.175, 0.125, 0}, {0.2, 0.125, 0},
  };

  std::vector<double> imgPoints{535.915733961632,
                                0,
                                342.2831547330837,
                                0,
                                535.915733961632,
                                235.5708290978817,
                                0,
                                0,
                                1};
  const cv::Mat1d cameraMatrix{3, 3, imgPoints.data()};

  std::vector<double> distCoeffsVec{-0.2663726090966068, -0.03858889892230465,
                                    0.001783194704285296,
                                    -0.0002812210044111547, 0.2383915308087849};
  const cv::Mat1d distCoeffs{5, 1, distCoeffsVec.data()};

  Mat rvec1, tvec1;
  solvePnP(objectPoints, corners1, cameraMatrix, distCoeffs, rvec1, tvec1);

  Mat rvec2, tvec2;
  solvePnP(objectPoints, corners2, cameraMatrix, distCoeffs, rvec2, tvec2);

  Mat R1, R2;
  Rodrigues(rvec1, R1);
  Rodrigues(rvec2, R2);

  Mat R_1to2, t_1to2;
  computeC2MC1(R1, tvec1, R2, tvec2, R_1to2, t_1to2);

  Mat rvec_1to2;
  Rodrigues(R_1to2, rvec_1to2);

  Mat normal = (Mat_<double>(3, 1) << 0, 0, 1);
  Mat normal1 = R1 * normal;
  Mat origin(3, 1, CV_64F, Scalar(0));
  Mat origin1 = R1 * origin + tvec1;
  double d_inv1 = 1.0 / normal1.dot(origin1);

  Mat homography_euclidean = computeHomography(R_1to2, t_1to2, d_inv1, normal1);
  Mat homography = cameraMatrix * homography_euclidean * cameraMatrix.inv();
  homography /= homography.at<double>(2, 2);
  homography_euclidean /= homography_euclidean.at<double>(2, 2);

  vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
  int solutions = decomposeHomographyMat(homography, cameraMatrix, Rs_decomp,
                                         ts_decomp, normals_decomp);
  cout << "Decompose homography matrix computed from the camera displacement:"
       << endl
       << endl;

  for (int i = 0; i < solutions; i++) {
    double factor_d1 = 1.0 / d_inv1;
    Mat rvec_decomp;
    Rodrigues(Rs_decomp[i], rvec_decomp);
    cout << "Solution " << i << ":" << endl;
    cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
    cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
    cout << "tvec from homography decomposition: " << ts_decomp[i].t()
         << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
    cout << "tvec from camera displacement: " << t_1to2.t() << endl;
    cout << "plane normal from homography decomposition: "
         << normals_decomp[i].t() << endl;
    cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
  }

  Mat H = findHomography(corners1, corners2);
  solutions = decomposeHomographyMat(H, cameraMatrix, Rs_decomp, ts_decomp,
                                     normals_decomp);
  cout << "Decompose homography matrix estimated by findHomography():" << endl
       << endl;

  for (int i = 0; i < solutions; i++) {
    double factor_d1 = 1.0 / d_inv1;
    Mat rvec_decomp;
    Rodrigues(Rs_decomp[i], rvec_decomp);
    cout << "Solution " << i << ":" << endl;
    cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
    cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
    cout << "tvec from homography decomposition: " << ts_decomp[i].t()
         << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
    cout << "tvec from camera displacement: " << t_1to2.t() << endl;
    cout << "plane normal from homography decomposition: "
         << normals_decomp[i].t() << endl;
    cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  decomposeHomography();
  return 0;
}
