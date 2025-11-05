#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

std::string get_current_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&itt), "%Y%m%d_%H%M%S");
  return ss.str();
}

cv::Mat generate_checkerboard(int rows_num, int columns_num, int block_size,
                              const cv::Scalar& base_color) {
  block_size *= 4;
  int width = block_size * columns_num;
  int height = block_size * rows_num;

  cv::Scalar inv_color(255 - base_color[0], 255 - base_color[1],
                       255 - base_color[2]);
  cv::Mat checkerboard(height, width, CV_8UC3);

  for (int i = 0; i < height; i += block_size) {
    bool row_toggle = ((i / block_size) % 2 == 0);
    for (int j = 0; j < width; j += block_size) {
      bool col_toggle = ((j / block_size) % 2 == 0);
      cv::Scalar color = (row_toggle == col_toggle) ? base_color : inv_color;
      cv::rectangle(checkerboard, cv::Point(j, i),
                    cv::Point(j + block_size, i + block_size), color,
                    cv::FILLED);
    }
  }

  return checkerboard;
}

bool save_image_as_png(const std::string& filename, const cv::Mat& image,
                       const std::string& path) {
  try {
    if (!fs::exists(path)) {
      fs::create_directories(path);
    }
    std::string full_path = path + "/" + filename + ".png";
    return cv::imwrite(full_path, image);
  } catch (const std::exception& e) {
    std::cerr << "Error saving image: " << e.what() << std::endl;
    return false;
  }
}

cv::String params =
    "{ help h             |                | print usage }"
    "{ rows-grid-num r    | 6              | chessboard height }"
    "{ columns-grid-num c | 9              | chessboard width }"
    "{ block-size-px s    | 30             | chessboard square size in pixels }"
    "{ output-file-name o | Checkerboard_" +
    get_current_timestamp() +
    ".png | filename of a chessboard image }"
    "{ output-path d      | .              | path to a chessboard image }"
    "{ base-color b       | 255,255,255    | base color in format R,G,B }";

int main(int argc, char* argv[]) {
  cv::CommandLineParser parser(argc, argv, params);
  parser.about("Code for chessboard image generation.\n");

  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  int rows_num = parser.get<int>("rows-grid-num");
  std::cout << rows_num << std::endl;
  int columns_num = parser.get<int>("columns-grid-num");
  int block_size = parser.get<int>("block-size-px");
  std::string output_name = parser.get<cv::String>("output-file-name");
  std::string output_path = parser.get<cv::String>("output-path");
  std::string base_color_str = parser.get<cv::String>("base-color");

  if (rows_num <= 0 || columns_num <= 0) {
    std::cerr
        << "Error: rows and columns must be provided and greater than 0.\n";
    return 1;
  }

  // Parse base color
  int r, g, b;
  sscanf(base_color_str.c_str(), "%d,%d,%d", &r, &g, &b);
  cv::Scalar base_color(b, g, r);  // OpenCV uses BGR

  std::cout << "Generating checkerboard..." << std::endl;

  cv::Mat checkerboard =
      generate_checkerboard(rows_num, columns_num, block_size, base_color);

  if (save_image_as_png(output_name, checkerboard, output_path)) {
    std::cout << "Saved as PNG: "
              << fs::absolute(output_path + "/" + output_name) << std::endl;
  } else {
    std::cerr << "Failed to save image." << std::endl;
  }

  return 0;
}
