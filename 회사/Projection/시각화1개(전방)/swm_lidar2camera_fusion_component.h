/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "modules/perception/base/object.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/fusion/lib/interface/base_multisensor_fusion.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/proto/swm_lidar2camera_fusion_component_config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/direct_shared_memory/direct_shared_memory.h"
#include <map>

#include "opencv2/opencv.hpp"
#include "modules/perception/base/object_types.h"

#define VISUALIZATION

namespace apollo {
namespace perception {
namespace onboard {

using namespace apollo::cyber;
using namespace apollo::drivers;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class SwmLidar2cameraFusionComponent : public apollo::cyber::Component<apollo::drivers::PointCloud> {
 public:
  SwmLidar2cameraFusionComponent() = default;
  ~SwmLidar2cameraFusionComponent() = default;
  bool Init() override;

  // message(shard_ptr) → PointCloud
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& message) override;

 private:
  bool InitAlgorithmPlugin();

  // message → PointCloud
  // in_box_message, out_message → PerceptionObstacles
  bool InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_pcd_message,
                    const std::shared_ptr<PerceptionObstacles>& in_box_message,
                    const std::shared_ptr<PerceptionObstacles>& out_message);
  
 private:
  int test_cnt ;
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;

  std::string fusion_name_;
  std::string fusion_method_;
  std::string fusion_main_sensor_;

  bool object_in_roi_check_ = false;
  double radius_for_roi_object_check_ = 0;

  double box_width;
  double offset_top, offset_bottom;
  double offset_front, offset_width;

  // writer_ → Writer(PerceptionObstacles)
  // box_reader_ → Reader(PerceptionObstacles)
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Reader<PerceptionObstacles>> box_reader_;

  std::string camera_name_;
  std::string lidar_name_;

  // { 값, ..., 값 }
  std::vector<std::string> camera_names_; 
  std::vector<std::string> lidar_names_; 

  // 사전: {string : Matrix4d, ..., string : Matrix4d }
  // { camera_name:cam_extrinsic, ..., camera_name:cam_extrinsic }
  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;

  // { camera_name:intrinsic, ..., camera_name:intrinsic }
  std::map<std::string, Eigen::Matrix3f> intrinsic_map_;

  // { camera_name:P, ..., camera_name:P }
  std::map<std::string, Eigen::Matrix<double, 3, 4>> resultMatrix_map_;

  std::vector<float> box_centers; 

  struct alignas(16) PointIL {
    float x = 0;
    float y = 0;
    float z = 0;
    float distance =0;
    int id = -1;
    base::ObjectType label = base::ObjectType::UNKNOWN;
    base::ObjectSubType sub_label = base::ObjectSubType::UNKNOWN;
  };

  // box_roi_pcd_msgs_ → {x,y,z,id,label,sub_label}
  std::vector<std::shared_ptr<PointIL>> box_roi_pcd_msgs_;
  std::vector<std::shared_ptr<PointIL>> box_near_pcd_msgs_;

  // visualize
  #ifdef VISUALIZATION
  // { box_id:number of points, ..., box_id:number of points }
  std::unordered_map<unsigned int, unsigned int> points_per_box;

  std::vector<cv::Scalar> colors = {
  cv::Scalar(0, 0, 255), 
  cv::Scalar(0, 255, 0),
  cv::Scalar(255, 0, 0),
  cv::Scalar(255,255,0),
  cv::Scalar(255,0,255),

  cv::Scalar(255, 30, 157), 
  cv::Scalar(20, 150, 255), 
  cv::Scalar(26, 94, 174),  
  cv::Scalar(173, 239, 120),
  cv::Scalar(130, 130, 130),

  cv::Scalar(131, 70, 130), 
  cv::Scalar(241, 30, 178),
  cv::Scalar(69, 119, 154), 
  cv::Scalar(85, 160, 230), 
  cv::Scalar(41, 109, 229), 

  cv::Scalar(100, 100, 163),
  cv::Scalar(60, 190, 255), 
  cv::Scalar(137, 86, 215)
  };
  #endif

};

CYBER_REGISTER_COMPONENT(SwmLidar2cameraFusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
