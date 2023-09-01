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

#include <Eigen/Dense>

#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "modules/perception/base/object.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/fusion/lib/interface/base_multisensor_fusion.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/proto/swm_bp_cam_fusion_component_config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/direct_shared_memory/direct_shared_memory.h"
#include <map>
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/base/object_types.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/bp_cam_fusion/lib/interface/base_tracker.h"
#include "modules/perception/bp_cam_fusion/lib/interface/base_bp_cam_fusion_obstacle_perception.h"
#include "opencv2/opencv.hpp"

#include "modules/drivers/lidar/common/CVC_cluster/CVC_cluster.h"

// #define shm_bp 

namespace apollo {
namespace perception {
namespace onboard {

using namespace apollo::cyber;
using namespace apollo::drivers;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class SwmBpCamFusionComponent : public apollo::cyber::Component<drivers::PointCloud>  {
 public:
  SwmBpCamFusionComponent()  = default;
  ~SwmBpCamFusionComponent() = default;
  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::PointCloud>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_pcd_message,
                    const std::shared_ptr<PerceptionObstacles>& in_box_message,
                    const std::shared_ptr<SensorFrameMessage>& out_message);
  void CalcHeading(cv::Point2f& point1, cv::Point2f& point2, Eigen::Affine3d& pose, float& heading);
  void CalDiffAngle(float angle1, float angle2, float& diff_angle);

  void CvcClustering(const Eigen::MatrixX3f target_nongroundd);

  std::shared_ptr<PointCloud> box_pcd_data ;
  std::shared_ptr<Writer<PointCloud>> box_bp_writer_;


  TransformWrapper lidar2world_trans_;
  float lidar_query_tf_offset_ = 20.0f;
  std::string lidar2novatel_tf2_child_frame_id_;

  int nearest_obstacle_id = 0;

  std::shared_ptr<bp_cam_fusion::BaseBpCamFusionObstaclePerception> bp_cam_fusion_perception_;

  base::SensorInfo lidar_info_;
  base::SensorInfo camera_info_;
  bool viz_switch = false;
  double occlusion_filter = 5;
  
  #ifdef shm_bp 
  // shm start
  std::thread process_thread_;
  std::shared_ptr<DirectSharedMemory> shared_memory_;
  // uint8_t* dataPtr;
  size_t point_size = 332800; // 1frame pointSize
  // SimplePoint aPoint; // TODO :: rename structure name
  // size_t aPointSize = sizeof(SimplePoint);

  size_t at128_lidar_num = 4;
  size_t at128_independent_memory_offset = 83200;
  std::deque<bool> at128_device_state = {true, true, true, true};
  std::vector<std::deque<float>> at128_point_size_buff;
  apollo::cyber::_PointXYZIT* dataPtr;
  //RSBP
  size_t rsbp_lidar_num = 4;
  size_t rsbp_start_pointer_offset = 332800;
  size_t rsbp_independent_memory_offset = 83200;
  size_t rsbp_lidar_position = 0;

  bool bpearl_device_state[4] = {true, true, true, true};
  uint64_t lidar_timestamp_array[4] = {0, 0, 0, 0};
  uint64_t prev_lidar_timestamp_array[4] = {0, 0, 0, 0};
  size_t device_check_cnt[4] = {0, 0, 0, 0};


  // Thread loop running flag
  bool isRunning_;

  void Stop() {
      isRunning_ = false;
      if (process_thread_.joinable()) {
        process_thread_.join();
      }
    }
  // shm end
  #endif

  int test_cnt ;
  static std::mutex s_mutex_;
  static uint32_t seq_num_;

  std::string sub_lidar_fusion_name;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> writer_;
  std::shared_ptr<apollo::cyber::Reader<PointCloud>> rsbp_reader_;
  std::shared_ptr<apollo::cyber::Reader<PerceptionObstacles>> box_reader_;

  std::string camera_name_; 
  std::string lidar_name_; 

  std::vector<std::string> camera_names_; 
  std::vector<std::string> lidar_names_; 

  EigenMap<std::string, Eigen::Matrix4d> extrinsic_map_;
  EigenMap<std::string, Eigen::Matrix3f> intrinsic_map_;
  EigenMap<std::string, Eigen::Matrix<double, 3, 4>> resultMatrix_map_;
  EigenMap<std::string, Eigen::Matrix<double, 3, 4>> imu2cameraMatrix_map_;

  std::map<int, int> over_box;
  std::map<int, float> mean_box_dis;

  double f_x;

  std::vector<float> box_centers; 


  struct alignas(16) PointIL {
    float x = 0;
    float y = 0;
    float z = 0;
    float distance =0;
    int id = -1;
    base::ObjectType label = base::ObjectType::UNKNOWN;
    base::ObjectSubType sub_label = base::ObjectSubType::UNKNOWN;
    double box_xlength = 0;
    double box_ylength = 0;
  };

  struct alignas(16) Box_Point_check {
    float x = 0;
    float y = 0;
    float z = 0;
    float distance =0;
    std::vector<float> id ;
    std::vector<base::ObjectType> label ;
    std::vector<base::ObjectSubType> sub_label ;
    std::vector<double> box_xlength ;
    std::vector<double> box_ylength ;
  };

  struct BoxInfo {
    int id = -1;
    std::vector<cv::Point2f> points;
    cv::Point2f box_center;
    cv::Point2f box_point1;
    cv::Point2f box_point2;
    cv::Point2f box_point3;
    cv::Point2f box_point4;
    bool lshape_state;
  };
  std::vector<std::shared_ptr<BoxInfo>> BoxesInfo;
  float lane_heading;

  // std::shared_ptr<PointIL> box_roi_pcd_msg_;
  std::vector<std::shared_ptr<PointIL>> box_roi_pcd_msgs_;
  std::vector<std::shared_ptr<PointIL>> box_near_pcd_msgs_;
  std::vector<std::shared_ptr<PointIL>> box_roi_pcd_msgs_erase;
  std::vector<std::shared_ptr<PointIL>> box_roi_pcd_msgs_dummy;
  std::vector<std::shared_ptr<PointIL>> box_roi_pcd_msgs_cvc;

  std::vector<std::shared_ptr<Box_Point_check>> box_checks;

  std::vector<double> car_gradient_vec;
  std::vector<double> van_gradient_vec;
  std::vector<double> truck_gradient_vec;
  std::vector<double> bus_gradient_vec;
  std::vector<double> cyclist_gradient_vec;
  std::vector<double> motorcyclist_gradient_vec;
  std::vector<double> tricyclist_gradient_vec;
  std::vector<double> pedestrian_gradient_vec;
  std::vector<double> trafficon_gradient_vec;

  double car_gradient = 1.55;
  double van_gradient = 2.2;
  double truck_gradient = 2.2;
  double bus_gradient = 3.0;
  double cyclist_gradient = 1.6;
  double motorcyclist_gradient = 1.6;
  double tricyclist_gradient = 1.6;
  double pedestrian_gradient = 1.6;
  double trafficon_gradient = 1.0;

  uint16_t top_view_width = 2000;
  uint16_t top_view_height = 2000;
  uint8_t x_range = 50;
  uint8_t y_range = 30;

  // cv::Mat top_view_img ;
  // cv::Mat front_view_img ;
  // cv::Mat top_view_img_init ;
  // cv::Mat front_view_img_init ;

  double camera2imu_origin_x ;
  double camera2imu_origin_y ;
  // double angle_rad ;
  double angle_l_rad ;
  double angle_r_rad ;
  float camera_angle = 80;

  double y_r ;
  double y_r_thd ;

  double y_l;
  double y_l_thd;
  
  std::vector<cv::Scalar> colors = {
  cv::Scalar(0, 0, 255), 
  // cv::Scalar(0, 255, 0),
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

  std::vector<double> anchor_width_vec;
  std::vector<double> anchor_length_vec;
  std::vector<double> anchor_height_vec;

  map::HDMapInput* hdmap_input_ = nullptr;
  Eigen::Vector3d lane_direction;
};


CYBER_REGISTER_COMPONENT(SwmBpCamFusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo