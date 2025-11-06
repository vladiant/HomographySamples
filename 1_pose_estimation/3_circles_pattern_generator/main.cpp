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
                              const cv::Scalar& base_color, int radius) {
  int width = block_size * columns_num;
  int height = block_size * rows_num;

  cv::Scalar inv_color(255 - base_color[0], 255 - base_color[1],
                       255 - base_color[2]);
  cv::Mat checkerboard(height, width, CV_8UC3,
                       base_color);  // Default background to inverted color

  for (int i = 0; i < rows_num; ++i) {
    for (int j = 0; j < columns_num; ++j) {
      bool is_white = (i + j) % 2 == 0;

      int x = j * block_size;
      int y = i * block_size;

      if (!is_white) {
        // Draw circle on inverted background
        cv::Point center(x + block_size / 2, y + block_size / 2);
        cv::circle(checkerboard, center, radius, inv_color, cv::FILLED);
      }
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

int main(int argc, char* argv[]) {
  // Default values
  int rows_num = 6;
  int columns_num = 9;
  int block_size = 120;
  std::string output_name = "Checkerboard_" + get_current_timestamp();
  std::string output_path = ".";
  std::string base_color_str = "(255,255,255)";

  // Parse arguments (simplified version)
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-r" || arg == "--rows-grid-num") && i + 1 < argc) {
      rows_num = std::stoi(argv[++i]);
    } else if ((arg == "-c" || arg == "--columns-grid-num") && i + 1 < argc) {
      columns_num = std::stoi(argv[++i]);
    } else if ((arg == "-s" || arg == "--block-size-px") && i + 1 < argc) {
      block_size = std::stoi(argv[++i]);
    } else if ((arg == "-o" || arg == "--output-file-name") && i + 1 < argc) {
      output_name = argv[++i];
    } else if ((arg == "-d" || arg == "--output-path") && i + 1 < argc) {
      output_path = argv[++i];
    } else if ((arg == "-b" || arg == "--base-color") && i + 1 < argc) {
      base_color_str = argv[++i];
    }
  }

  if (rows_num <= 0 || columns_num <= 0) {
    std::cerr
        << "Error: rows and columns must be provided and greater than 0.\n";
    return 1;
  }

  // Parse base color
  int r, g, b;
  sscanf(base_color_str.c_str(), "(%d,%d,%d)", &r, &g, &b);
  cv::Scalar base_color(b, g, r);  // OpenCV uses BGR

  std::cout << "Generating checkerboard..." << std::endl;

  cv::Mat checkerboard = generate_checkerboard(
      rows_num, columns_num, block_size, base_color, block_size / 2);

  if (save_image_as_png(output_name, checkerboard, output_path)) {
    std::cout << "Saved as PNG: "
              << fs::absolute(output_path + "/" + output_name + ".png")
              << std::endl;
  } else {
    std::cerr << "Failed to save image." << std::endl;
  }

  return 0;
}
