#include <okvis/OakD.hpp>
#include <iostream>

bool imageCallback(const okvis::Time &timestamp, const std::map<size_t, cv::Mat> &frame, const std::map<size_t, cv::Mat> &depthFrame) {
  std::cout << "Processing frame..." << std::endl;
  cv::imshow("left", frame.at(0));
  cv::imshow("right", frame.at(1));
//   cv::imshow("rgb", frame.at(2));
  cv::waitKey(1);
    return true;
}

int main() {
  okvis::OakD oakd(false);
  oakd.startStreaming();
  oakd.setImagesCallback(imageCallback);

//   // Connect to device and start pipeline
//   dai::Device device(oakd.pipeline_);
//   device.getOutputQueue("syncImgs", 8, false)->addCallback([](std::shared_ptr<dai::ADatatype> data){
//      std::cout << "Processing frame..." << std::endl;
//   });
  // empty loop
  while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // //   oakd.setIrRes(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    // oakd.stopStreaming();
    //   std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // //   oakd.setIrRes(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    // oakd.startStreaming();
  }
  
}