#include <okvis/OakD.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

struct ImuData {
  uint32_t secs;
  uint32_t nsecs;
  double accelerometers[3];
  double gyroscopes[3];
};

static std::string getFormattedTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  return ss.str();
}

static uint64_t counter = 0;
static std::string dataDir = "/home/freissml/logs/oakd_imgcal";
static std::string imagesDir = dataDir + "/images";
static std::string imuFilePath = dataDir + "/imu_data_" + getFormattedTimestamp() + ".bin";
static std::string csvFilePath = dataDir + "/img_data_" + getFormattedTimestamp() + ".csv";

bool imageCallback(const okvis::Time &timestamp, const std::map<size_t, cv::Mat> &frame, const std::map<size_t, cv::Mat> &depthFrame) {
  // cv::imwrite(imagesDir + "/frame_" + std::to_string(counter) + "_left.png", frame.at(0));
  // cv::imwrite(imagesDir + "/frame_" + std::to_string(counter) + "_right.png", frame.at(1));
  // cv::imwrite(imagesDir + "/frame_" + std::to_string(counter) + "_rgb.png", frame.at(2));

  // std::ofstream csvFile(csvFilePath, std::ios::app);
  // csvFile << timestamp.sec << "," << timestamp.nsec << ","
  //         << imagesDir + "/frame_" + std::to_string(counter) + "_left.png" << ","
  //         << imagesDir + "/frame_" + std::to_string(counter) + "_right.png" << ","
  //         << imagesDir + "/frame_" + std::to_string(counter) + "_rgb.png" << std::endl;
  // csvFile.close();
  cv::imshow("left", frame.at(0));
  cv::imshow("right", frame.at(1));
  cv::imshow("rgb", frame.at(2));
  cv::waitKey(1);
  counter++;
  return true;
}

bool imuCallback(const okvis::Time &timestamp, const Eigen::Vector3d &accelerometers, const Eigen::Vector3d &gyroscopes) {
  // std::cout << "IMU timestamp " << timestamp.sec << "." << std::setfill('0') << std::setw(9) << timestamp.nsec << std::endl;
  // static std::ofstream imuFile(imuFilePath, std::ios::binary | std::ios::app);
  // ImuData data;
  // data.secs = timestamp.sec;
  // data.nsecs = timestamp.nsec;
  // for (int i = 0; i < 3; ++i) {
  //   data.accelerometers[i] = static_cast<double>(accelerometers[i]);
  //   data.gyroscopes[i] = static_cast<double>(gyroscopes[i]);
  // }
  // imuFile.write(reinterpret_cast<const char*>(&data), sizeof(ImuData));
  return true;
}

int main() {
  okvis::OakD oakd(true, 60., 60., 400);
  oakd.startStreaming();
  // oakd.setImagesCallback(imageCallback);
  oakd.setImuCallback(imuCallback);

  // empty loop
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // oakd.stopStreaming();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // oakd.startStreaming();
  }
  
}