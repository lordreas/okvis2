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
    bool enableRgb, const dai::ColorCameraProperties::SensorResolution rgbRes, const dai::MonoCameraProperties::SensorResolution irRes, const float rgbFps, const float irFps, const uint8_t imuRate)
    : pipeline_{}, pipelineInitialized_{false}, streaming_{false}, enableRgb_{enableRgb}, rgbRes_{rgbRes}, irRes_{irRes}, rgbFps_{rgbFps}, irFps_{irFps}, imuRate_{imuRate} {
  setupPipeline_();
}

OakD::~OakD() {
  if(streaming_) {
    stopStreaming();
  }
}

void OakD::setIrRes(dai::MonoCameraProperties::SensorResolution irRes){
  irRes_ = irRes;
  
  bool wasStreaming = streaming_;
  if (wasStreaming)
    stopStreaming(); // TODO maybe this isn't necessary
  
  setupPipeline_();
  
  if (wasStreaming)
    startStreaming();// TODO maybe this isn't necessary
  
}

void OakD::setRgbRes(dai::ColorCameraProperties::SensorResolution rgbRes){
  rgbRes_ = rgbRes;
  
  bool wasStreaming = streaming_;
  if (wasStreaming)
    stopStreaming(); // TODO maybe this isn't necessary
  
  setupPipeline_();
  
  if (wasStreaming)
    startStreaming();// TODO maybe this isn't necessary
  
}

void OakD::setIrFps(float irFps){
  irFps_ = irFps;
  
  bool wasStreaming = streaming_;
  if (wasStreaming)
    stopStreaming(); // TODO maybe this isn't necessary
  
  setupPipeline_();
  
  if (wasStreaming)
    startStreaming();// TODO maybe this isn't necessary
  
}

void OakD::setRgbFps(float rgbFps){
  rgbFps_ = rgbFps;
  
  bool wasStreaming = streaming_;
  if (wasStreaming)
    stopStreaming(); // TODO maybe this isn't necessary
  
  setupPipeline_();
  
  if (wasStreaming)
    startStreaming();// TODO maybe this isn't necessary
  
}

void OakD::processFrame(std::shared_ptr<dai::ADatatype> data){
  std::shared_ptr<dai::MessageGroup> messageGroup = std::dynamic_pointer_cast<dai::MessageGroup>(data);
  
  std::map<size_t, cv::Mat> outFrame;
  outFrame[0] = messageGroup->get<dai::ImgFrame>("left")->getCvFrame();
  outFrame[1] = messageGroup->get<dai::ImgFrame>("right")->getCvFrame();
  if (enableRgb_) {
    outFrame[2] = messageGroup->get<dai::ImgFrame>("rgb")->getCvFrame();
  }
  std::chrono::steady_clock::time_point oakTs = messageGroup->get<dai::ImgFrame>("left")->getTimestamp();
  okvis::Time timestamp(oakTs.time_since_epoch().count());

  std::cout << "Image dimensions of left image: " << outFrame[0].size() << std::endl;
  std::cout << "Image dimensions of right image: " << outFrame[1].size() << std::endl;

  for (auto &imagesCallback : imagesCallbacks_) {
    imagesCallback(timestamp, outFrame, std::map<size_t, cv::Mat>());
  }


}

void OakD::processImu(std::shared_ptr<dai::ADatatype> data){

  // ImuMeasurement imuMeasurement{
    
  // }

  // for (auto &imuCallback : imuCallbacks_) {
  //   imuCallback(timestamp, Eigen::Vector3d(), Eigen::Vector3d());
  // }

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
  imgQueue_->close();
  imuQueue_->close();
  streaming_ = false;
}


bool OakD::isStreaming(){
  return streaming_;
}




// bool OakD::startStreaming_(const cv::Size& irSize, const uint irFps, const cv::Size& rgbSize, const uint rgbFps){

// }


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
  std::shared_ptr<dai::node::StereoDepth> stereo = pipeline_.create<dai::node::StereoDepth>();
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
  sync->setSyncThreshold(std::chrono::milliseconds(10));
  imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, imuRate_);
  xoutSync->setStreamName("syncImgs");
  xoutImu->setStreamName("imu");

  if (enableRgb_) {
    std::shared_ptr<dai::node::ColorCamera> colorCamera = pipeline_.create<dai::node::ColorCamera>();
    colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCamera->setResolution(rgbRes_);
    colorCamera->setFps(rgbFps_);
    colorCamera->video.link(sync->inputs["rgb"]);
  }

  // Make connections in pipeline graph
  monoLeft->out.link(stereo->left);
  monoRight->out.link(stereo->right);
  stereo->rectifiedLeft.link(sync->inputs["left"]);
  stereo->rectifiedRight.link(sync->inputs["right"]);
  imu->out.link(xoutImu->input);
  sync->out.link(xoutSync->input);


  pipelineInitialized_ = true;
  
}

}