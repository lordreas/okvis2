/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file OakD.cpp
 * @brief Source file for the OakD class.
 * @author Leonard Freissmuth
 */


#include <okvis/OakD.hpp>

namespace okvis{

OakD::OakD(
    bool enableRgb, const float rgbFps, const float irFps, const uint32_t imuRate, const dai::ColorCameraProperties::SensorResolution rgbRes, const dai::MonoCameraProperties::SensorResolution irRes)
    : pipeline_{}, pipelineInitialized_{false}, streaming_{false}, enableRgb_{enableRgb}, rgbRes_{rgbRes}, irRes_{irRes}, rgbFps_{rgbFps}, irFps_{irFps}, imuRate_{imuRate} {
  setupPipeline_();
}

OakD::~OakD() {
  stopStreaming();
}

void OakD::processFrame(std::shared_ptr<dai::ADatatype> data){
  // check if the image queue is initialized
  if (!imgQueue_) {
    return;
  }

  // check if streaming
  if (!streaming_) {
    return;
  }

  // discard warmup frames
  if (warmupCounter_ < numWarnmupFrames_) {
    warmupCounter_++;
    return;
  }

  std::shared_ptr<dai::MessageGroup> messageGroup = std::dynamic_pointer_cast<dai::MessageGroup>(data);
  
  std::map<size_t, cv::Mat> outFrame;
  auto leftFrame = messageGroup->get<dai::ImgFrame>("left");
  if (!leftFrame) {
    LOG(ERROR) << "Left frame not found in message group.";
    return;
  }
  auto rightFrame = messageGroup->get<dai::ImgFrame>("right");
  if (!rightFrame) {
    LOG(ERROR) << "Right frame not found in message group.";
    return;
  }
  outFrame[0] = leftFrame->getFrame(true);
  outFrame[1] = rightFrame->getFrame(true);
  if (enableRgb_) {
    auto rgbFrame = messageGroup->get<dai::ImgFrame>("rgb");
    if (!rgbFrame) {
      LOG(ERROR) << "RGB frame not found in message group.";
      return;
    }
    outFrame[2] = rgbFrame->getFrame(true);
  }
  
  // timestamps of left and right frame are hardware synced (sub 1 ms latency) and the 
  // rgb frame is software synced (sub 1/2 frame latency)
  // so we can use the left frame timestamp as the timestamp for all frames
  okvis::Time timestamp(std::chrono::duration_cast<std::chrono::duration<float>>(leftFrame->getTimestamp().time_since_epoch()).count());

  for (auto &imagesCallback : imagesCallbacks_) {
    imagesCallback(timestamp, outFrame, std::map<size_t, cv::Mat>());
  }
}

void OakD::processImu(std::shared_ptr<dai::ADatatype> data){
  // check if the imu is initialized
  if (!imuQueue_) {
    return;
  }
  std::shared_ptr<dai::IMUData> imuData = std::dynamic_pointer_cast<dai::IMUData>(data);

  for (auto &packet : imuData->packets) {
    // std::cout << "- - - - - - - - - - - - - - - - - - - - - - - - -" << std::endl;
    // std::cout << "Timestamp acc: " << packet.acceleroMeter.getTimestamp().time_since_epoch().count() << std::endl;
    // std::cout << "Timestamp gyr: " << packet.gyroscope.getTimestamp().time_since_epoch().count() << std::endl;
    // std::cout << "Accuracy acc: " << static_cast<int>(packet.acceleroMeter.accuracy) << std::endl;
    // std::cout << "Accuracy gyr: " << static_cast<int>(packet.gyroscope.accuracy) << std::endl;
    uint64_t nanoseconds = packet.acceleroMeter.getTimestamp().time_since_epoch().count();
    uint32_t secs = nanoseconds / 1000000000;
    uint32_t nsecs = (nanoseconds % 1000000000);
    okvis::Time timestamp(secs, nsecs);
    Eigen::Vector3d gyr(packet.gyroscope.x, packet.gyroscope.y, packet.gyroscope.z);
    Eigen::Vector3d acc(packet.acceleroMeter.x, packet.acceleroMeter.y, packet.acceleroMeter.z);
    for (auto &imuCallback : imuCallbacks_) {
      imuCallback(timestamp, acc, gyr);
    }
  }

}

bool OakD::startStreaming(){
  if (!pipelineInitialized_) {
    LOG(ERROR) << "Pipeline not initialized. Cannot start streaming.";
    return false;
  }

  LOG(INFO) << "Starting pipeline...";

  device_ = std::make_unique<dai::Device>(pipeline_);

  // check if the device is connected
  // std::cout << "Number of connected devices: " << device_->getAllConnectedDevices().size() << std::endl;
  // if (device_->getAllConnectedDevices().size() < 1) {
  //     LOG(ERROR) << "No device connected";
  //     return false;
  // }

  imgQueue_ = device_->getOutputQueue("syncImgs", 8, false);
  imgQueue_->addCallback(std::bind(&OakD::processFrame, this, std::placeholders::_1));

  imuQueue_ = device_->getOutputQueue("imu", 8, false);
  imuQueue_->addCallback(std::bind(&OakD::processImu, this, std::placeholders::_1));

  streaming_ = true;
  return true;
}

bool OakD::stopStreaming(){
  if (!streaming_) {
      return true;
  }
  LOG(INFO) << "Stopping pipeline...";

  // Remove callbacks on IMU and image queues
  if (imgQueue_) {
      imgQueue_->removeCallback(0);
      imgQueue_->close();
      imgQueue_.reset();
  }

  if (imuQueue_) {
      imuQueue_->removeCallback(0);
      imuQueue_->close();
      imuQueue_.reset();
  }

  if (device_) {
      device_->close();
      device_.reset();
  }

  streaming_ = false;
  return true;

}

bool OakD::isStreaming(){
  return streaming_;
}

void OakD::setupPipeline_(){

  if(pipelineInitialized_){
    // If pipeline already exists, just update camera properties (resolution, frame rate)
    for (std::shared_ptr<dai::Node> &node : pipeline_.getAllNodes()){
      if (node->getName() == "ColorCamera"){
        std::shared_ptr<dai::node::ColorCamera> colorCamera = std::dynamic_pointer_cast<dai::node::ColorCamera>(node);
        colorCamera->setResolution(rgbRes_);
        colorCamera->setFps(rgbFps_);
      } else if (node->getName() == "MonoCamera"){
        std::shared_ptr<dai::node::MonoCamera> monoCamera = std::dynamic_pointer_cast<dai::node::MonoCamera>(node);
        monoCamera->setResolution(irRes_);
        monoCamera->setFps(irFps_);
      } 
    }
    return;
  }
  
  // Define sources and outputs
  std::shared_ptr<dai::node::MonoCamera> monoLeft = pipeline_.create<dai::node::MonoCamera>();
  std::shared_ptr<dai::node::MonoCamera>  monoRight = pipeline_.create<dai::node::MonoCamera>();
  // std::shared_ptr<dai::node::StereoDepth> stereo = pipeline_.create<dai::node::StereoDepth>();
  std::shared_ptr<dai::node::Sync> sync = pipeline_.create<dai::node::Sync>();
  std::shared_ptr<dai::node::IMU> imu = pipeline_.create<dai::node::IMU>();
  std::shared_ptr<dai::node::XLinkOut> xoutSync = pipeline_.create<dai::node::XLinkOut>();
  std::shared_ptr<dai::node::XLinkOut> xoutImu = pipeline_.create<dai::node::XLinkOut>();
  
  // setup camera and link properties
  monoLeft->setCamera("left");
  monoLeft->setResolution(irRes_);
  monoLeft->setFps(irFps_);
  monoRight->setCamera("right");
  monoRight->setResolution(irRes_);
  monoRight->setFps(irFps_);
  sync->setSyncThreshold(std::chrono::milliseconds(uint16_t(1000 / std::min(irFps_, rgbFps_))));
  imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, imuRate_);
  // imu->setBatchReportThreshold(1);
  // imu->setMaxBatchReports(1);
  xoutSync->setStreamName("syncImgs");
  xoutImu->setStreamName("imu");

  if (enableRgb_) {
    std::shared_ptr<dai::node::ColorCamera> colorCamera = pipeline_.create<dai::node::ColorCamera>();
    colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCamera->setResolution(rgbRes_);
    colorCamera->setFps(rgbFps_);
    colorCamera->isp.link(sync->inputs["rgb"]);
  }

  // Make connections in pipeline graph
  // monoLeft->out.link(stereo->left);
  // monoRight->out.link(stereo->right);
  // stereo->rectifiedLeft.link(sync->inputs["left"]);
  // stereo->rectifiedRight.link(sync->inputs["right"]);
  monoLeft->out.link(sync->inputs["left"]);
  monoRight->out.link(sync->inputs["right"]);
  imu->out.link(xoutImu->input);
  sync->out.link(xoutSync->input);


  pipelineInitialized_ = true;
  
}

}