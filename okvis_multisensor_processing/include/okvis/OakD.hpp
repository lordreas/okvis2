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
 * @file OakD.hpp
 * @brief Header file for the OakD class.
 * @author Leonard Freissmuth
 */

#ifndef INCLUDE_OKVIS_OAKD_HPP_
#define INCLUDE_OKVIS_OAKD_HPP_

#include <okvis/Measurements.hpp>
#include <okvis/ViSensorBase.hpp>
#include <glog/logging.h>
#include <depthai/depthai.hpp>

namespace okvis {
class OakD : public ViSensorBase {
public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  /// @brief Alternative constructor. Won't start the streaming.
  /// @param enableRgb Enable RGB camera?
  /// @param rgbRes Camera resolution of RGB cameras (as defined in dai::ColorCameraProperties::SensorResolution).
  /// @param irRes Camera resolution of infrared cameras (as defined in dai::MonoCameraProperties::SensorResolution).
  /// @param rgbFps RGB camera frame rate in (continusous of) fps.
  /// @param irFps IR camera frame rate in (continusous of) fps.
  /// @param imuRate IMU rate in Hz.
  OakD(bool enableRgb = false,
    const float rgbFps = 10.0f,
    const float irFps = 10.0f,
    const uint8_t imuRate = 200,
    const dai::ColorCameraProperties::SensorResolution rgbRes = dai::ColorCameraProperties::SensorResolution::THE_800_P,
    const dai::MonoCameraProperties::SensorResolution irRes = dai::MonoCameraProperties::SensorResolution::THE_800_P);
  
  /// @brief Destructor. Will also stop the streaming, if started.
  virtual ~OakD();

  /// @brief Specify IR image size. \warning Use eligible values.
  /// @param width Image width.
  /// @param height Image height.
  void setIrRes(dai::MonoCameraProperties::SensorResolution irRes);
  
  /// @brief Specify RGB image size. \warning Use eligible values.
  /// @param width Image width.
  /// @param height Image height.
  void setRgbRes(dai::ColorCameraProperties::SensorResolution rgbRes);   
  
  /**
   * @brief Set the frame rate for the IR camera.
   * @param irFps IR camera frame rate.
   */
  void setIrFps(float irFps);
  
  /**
   * @brief Set the frame rate for the RGB camera.
   * @param rgbFps RGB camera frame rate.
   */
  void setRgbFps(float rgbFps);

  /// \brief Process a frame.
  /// \param data the data block containing two (or 3 with rgb) synced frames.
  virtual void processFrame(std::shared_ptr<dai::ADatatype> data);

  /// \brief Process a frame.
  /// \param data the data block containing imu measurements.
  virtual void processImu(std::shared_ptr<dai::ADatatype> data);

  /// @brief Starts streaming.
  /// @return True, if successful
  bool startStreaming() override;

  /// @brief Stops streaming.
  /// @return True, if successful
  bool stopStreaming() override final;

  /// @brief Check if currently streaming.
  /// @return True, if streaming.
  bool isStreaming() override final;

// protected:
//   bool startStreaming_(
//     const dai::MonoCameraProperties::SensorResolution irRes,
//     const uint irFps,
//     const dai::ColorCameraProperties::SensrResolution rgbSize,
//     const uint rgbFps
//   );


// private:
  void setupPipeline_();

  dai::Pipeline pipeline_; ///< The pipeline coordinating data flow inside the camera.
  std::unique_ptr<dai::Device> device_; ///< The device object representing the OakD.
  std::shared_ptr<dai::DataOutputQueue> imgQueue_; ///< The queue for image data.
  std::shared_ptr<dai::DataOutputQueue> imuQueue_; ///< The queue for IMU data.
  bool pipelineInitialized_; ///< True if pipeline was initialized.
  std::atomic_bool streaming_; ///< True if streaming started.
  bool enableRgb_ = false; ///< RGB cam enabled?
  dai::ColorCameraProperties::SensorResolution rgbRes_; ///< RGB image Resolution.
  dai::MonoCameraProperties::SensorResolution irRes_; ///< IR image Resolution.
  float rgbFps_; ///< RGB camera FPS.
  float irFps_; ///< IR camera FPS.
  uint8_t imuRate_; ///< IMU rate.

  uint16_t numWarnmupFrames_ = 20; ///< Number of warmup frames.
  uint16_t warmupCounter_ = 0; ///< Counter for warmup frames.
};
}

#endif // INCLUDE_OKVIS_OAKD_HPP_