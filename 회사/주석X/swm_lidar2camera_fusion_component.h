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
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& message) override;

 private:
  bool InitAlgorithmPlugin();
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

  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Reader<PerceptionObstacles>> box_reader_;

  std::string camera_name_; 
  std::string lidar_name_; 

  std::vector<std::string> camera_names_; 
  std::vector<std::string> lidar_names_; 

  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;
  std::map<std::string, Eigen::Matrix3f> intrinsic_map_;
  std::map<std::string, Eigen::Matrix<double, 3, 4>> resultMatrix_map_;

  std::map<std::string, Eigen::Matrix<double, 3, 4>> extrinsic_distor_map_;


  struct alignas(16) PointIL {
    float x = 0;
    float y = 0;
    float z = 0;
    int id = -1;
    int label = 0;
    int sub_label = 0;
  };
  // std::shared_ptr<PointIL> box_roi_pcd_msg_;

  std::vector<std::shared_ptr<PointIL>> box_roi_pcd_msgs_;
  

};

CYBER_REGISTER_COMPONENT(SwmLidar2cameraFusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
