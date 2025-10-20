#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>
#include <vector>

using namespace cv;
using namespace std;

void loadExposureSeq(String, vector<Mat>&, vector<float>&);

// ./initial_example -input ./ 
int main(int argc, char** argv) {
  CommandLineParser parser(
      argc, argv,
      "{@input | | Input directory that contains images and exposure times. }");

  vector<Mat> images;
  vector<float> times;
  loadExposureSeq(parser.get<String>("@input"), images, times);

  // Align input images
  Ptr<AlignMTB> alignMTB = createAlignMTB();
  alignMTB->process(images, images);

  // Estimate camera response function (CRF)
  Mat response;
  Ptr<CalibrateCRF> calibrate = createCalibrateDebevec();
  calibrate->process(images, response, times);

  // Merge exposures to HDR image
  Mat hdr;
  Ptr<MergeExposures> merge_debevec = createMergeDebevec();
  merge_debevec->process(images, hdr, times, response);

  // Tonemap HDR image to obtain 24-bit color image
  Mat ldr;
  Ptr<Tonemap> tonemap = createTonemapDrago(2.2f);
  // Ptr<Tonemap> tonemap = createTonemapMantiuk(1.0f, 0.7f, 1.0f);
  // Ptr<Tonemap> tonemap = createTonemapReinhard(2.2f, 0.0f, 1.0f, 0.0f);
  tonemap->process(hdr, ldr);

  // Exposure fusion using Mertens method
  Mat fusion;
  Ptr<MergeMertens> merge_mertens = createMergeMertens();
  merge_mertens->process(images, fusion);

  imwrite("fusion.png", fusion * 255);
  imwrite("ldr.png", ldr * 255);
  imwrite("hdr.hdr", hdr);  // RADIANCE FORMAT

  return 0;
}

void loadExposureSeq(String path, vector<Mat>& images, vector<float>& times) {
  path = path + "/";
  ifstream list_file((path + "list.txt").c_str());
  string name;
  float val;
  while (list_file >> name >> val) {
    Mat img = imread(path + name);
    images.push_back(img);
    times.push_back(1 / val);
  }
  list_file.close();
}
