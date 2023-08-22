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
#include "modules/perception/onboard/component/swm_bp_cam_fusion_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"

#define CAR_WIDTH  1.84
#define CAR_LENGTH 4.96 
#define CAR_HEIGHT 1.72 
#define PEDESTRIAN_WIDTH  0.68
#define PEDESTRIAN_LENGTH 1.93
#define PEDESTRIAN_HEIGHT 1.29
#define BICYCLE_WIDTH  0.64
#define BICYCLE_LENGTH 0.49 
#define BICYCLE_HEIGHT 1.63 
#define PHI 3.14159265358979323846

namespace apollo {
namespace perception {
namespace onboard {

// std::atomic<uint32_t> SwmBpCamFusionComponent::seq_num_{0};
uint32_t SwmBpCamFusionComponent::seq_num_ = 0;
std::mutex SwmBpCamFusionComponent::s_mutex_;

static bool LoadExtrinsics(const std::string &yaml_file,
                           Eigen::Matrix4d *camera_extrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " does not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  camera_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*camera_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*camera_extrinsic)(0, 3) = tx;
  (*camera_extrinsic)(1, 3) = ty;
  (*camera_extrinsic)(2, 3) = tz;
  (*camera_extrinsic)(3, 3) = 1;
  return true;
}

std::string ObjectSubTypeToString(base::ObjectSubType subtype) {
  switch (subtype) {
    case base::ObjectSubType::UNKNOWN:
      return "UNKNOWN";
    case base::ObjectSubType::UNKNOWN_MOVABLE:
      return "UNKNOWN_MOVABLE";
    case base::ObjectSubType::UNKNOWN_UNMOVABLE:
      return "UNKNOWN_UNMOVABLE";
    case base::ObjectSubType::CAR:
      return "CAR";
    case base::ObjectSubType::VAN:
      return "VAN";
    case base::ObjectSubType::TRUCK:
      return "TRUCK";
    case base::ObjectSubType::BUS:
      return "BUS";
    case base::ObjectSubType::CYCLIST:
      return "CYCLIST";
    case base::ObjectSubType::MOTORCYCLIST:
      return "MOTORCYCLIST";
    case base::ObjectSubType::TRICYCLIST:
      return "TRICYCLIST";
    case base::ObjectSubType::PEDESTRIAN:
      return "PEDESTRIAN";
    case base::ObjectSubType::TRAFFICCONE:
      return "TRAFFICCONE";
    case base::ObjectSubType::MAX_OBJECT_TYPE:
      return "MAX_OBJECT_TYPE";
    default:
      return "UNKNOWN"; 
  }
}

bool SwmBpCamFusionComponent::Init() {
  SwmBpCamFusionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Swm Lidar2camera Fusion Component Configs: " << comp_config.DebugString();

  viz_switch = comp_config.viz_switch();
  occlusion_filter = comp_config.occlusion_filter(); 
  

  std::string camera_names_str = comp_config.camera_name();
  boost::algorithm::split(camera_names_, camera_names_str,
                          boost::algorithm::is_any_of(","));

  std::string lidar_names_str = comp_config.lidar_name();
  boost::algorithm::split(lidar_names_, lidar_names_str,
                          boost::algorithm::is_any_of(","));

  sub_lidar_fusion_name = comp_config.sub_lidar_fusion_name();
  if (!common::SensorManager::Instance()->GetSensorInfo(
          comp_config.sub_lidar_fusion_name(), &lidar_info_)) {
    AERROR << "Failed to get sensor info, sensor name: "
           << comp_config.sub_lidar_fusion_name();
    return false;
  }
  AERROR << "viz_switch : " << viz_switch;
  AERROR << "sub_lidar_fusion_name : " << sub_lidar_fusion_name;

  hdmap_input_= map::HDMapInput::Instance();
  ACHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";

  #ifndef shm_bp 
  // rsbp_reader_ = node_->CreateReader<apollo::drivers::PointCloud>(comp_config.input_bp_channel_name());
  // rsbp_reader_ = node_->CreateReader<apollo::drivers::PointCloud>("/apollo/sensor/rsbp/front/PointCloud2");
  // AERROR << comp_config.input_bp_channel_name();
  box_reader_ = node_->CreateReader<PerceptionObstacles>(comp_config.input_box_channel_name());
  
  #else

  shared_memory_ = std::make_shared<DirectSharedMemory>();

  #define _NOW() ({ \
      auto now = system_clock::now(); \
      auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count();  \
      ms; \
  })

  dataPtr = (_PointXYZIT*) shared_memory_->getPointer() + rsbp_start_pointer_offset;
  // dataPtr = (_PointXYZIT*) shared_memory_->getPointer();
  
  #endif

  if(!InitAlgorithmPlugin()) 
  {
    AERROR << "Failed to init algorithm plugin.";
    return false;
  }
  writer_ = node_->CreateWriter<SensorFrameMessage>(
      comp_config.output_obstacles_channel_name());
  // writer_ = node_->CreateWriter<SensorFrameMessage>("/perception/inner/PrefusedObjects");
  // writer_ = node_->CreateWriter<SensorFrameMessage>(comp_config.output_obstacles_channel_name());

  // box_bp_writer_ = node_->CreateWriter<PointCloud>("perception/test/box_in_bp_data");

  return true;
}

bool SwmBpCamFusionComponent::Proc(const std::shared_ptr<drivers::PointCloud>& message) {
  #ifndef shm_bp 
  box_reader_->Observe();
  auto in_box_message = box_reader_->GetLatestObserved();
  
  if (in_box_message == nullptr) {
   std::this_thread::sleep_for(std::chrono::seconds(1));
   in_box_message = box_reader_->GetLatestObserved();
  }

  if (in_box_message == nullptr) {
    return true;
  }
  #else
  // process_thread_ = std::thread([&](){
    // this->isRunning_ = true;
    // while(this->isRunning_) {
      // Get Points /////////////////
      std::shared_ptr<PointCloud> in_rsbp_message = std::make_shared<PointCloud>();

      uint64_t timestamp = 0;
      for (size_t lidar_num = rsbp_lidar_position; lidar_num < rsbp_lidar_position +1; lidar_num++) {
        auto tmp_dataPtr = dataPtr + rsbp_independent_memory_offset * lidar_num;
        prev_lidar_timestamp_array[lidar_num] = lidar_timestamp_array[lidar_num];
        size_t point_size = tmp_dataPtr->x;
        // AERROR << "point_size : " << point_size ;
        for (size_t point_index = 1; point_index < point_size; point_index++) {
          _PointXYZIT* _point = tmp_dataPtr + point_index;
          if (0 == _point->t) continue;
          if (lidar_timestamp_array[lidar_num] < _point->t) {
            lidar_timestamp_array[lidar_num] = _point->t;
          }
          if (bpearl_device_state[lidar_num]) {
            auto point = in_rsbp_message->add_point();
            point->set_x(_point->x);
            point->set_y(_point->y);
            point->set_z(_point->z);
            point->set_intensity(_point->i);
            point->set_timestamp(_point->t);
          }
        }
        // timestamp = in_rsbp_message->point(0).timestamp();
        timestamp = lidar_timestamp_array[lidar_num];
        bpearl_device_state[lidar_num] = true;
        if (lidar_timestamp_array[lidar_num] == 0 ||
            lidar_timestamp_array[lidar_num] == prev_lidar_timestamp_array[lidar_num]) {
          device_check_cnt[lidar_num]++;
          if (device_check_cnt[lidar_num] > 1) {
            bpearl_device_state[lidar_num] = false;
          }
        } else {
          device_check_cnt[lidar_num] = 0;
        }
      }

      // Set PointCloud header //////
      int size = in_rsbp_message->point_size();
      in_rsbp_message->set_width(size);
      in_rsbp_message->set_height(1);
      in_rsbp_message->mutable_header()->set_frame_id("velodyne128");
      in_rsbp_message->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
      in_rsbp_message->set_measurement_time((double)timestamp/1000000000);
      in_rsbp_message->mutable_header()->set_lidar_timestamp(timestamp);

      // box_bp_writer_->Write(in_rsbp_message);

      AERROR << "in_rsbp_message->point_size() : " << size;
    // }
  // });  
  #endif

  // auto out_message = std::make_shared<PerceptionObstacles>();
  auto out_message = std::make_shared<SensorFrameMessage>();
  // if(!InternalProc(in_rsbp_message, message, out_message)){
  if(!InternalProc(message, in_box_message, out_message)){
    AERROR << "cam2lidar fusion fail." ;
    return false;

  }
  writer_->Write(out_message);
  return true;
}

bool SwmBpCamFusionComponent::InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_pcd_message,
                                                  const std::shared_ptr<PerceptionObstacles>& in_box_message,
                                                  const std::shared_ptr<SensorFrameMessage>& out_message){                                         
  //tf
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_novatel = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      // in_pcd_message->measurement_time();// - lidar_query_tf_offset_ * 0.001;
      in_pcd_message->header().timestamp_sec()-0.1;//hoseob
  

  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose,
                                               &pose_novatel)) {
    return true;
  } 

  uint16_t top_view_width = 2000;
  uint16_t top_view_height = 2000;
  uint8_t x_range = 100;
  uint8_t y_range = 100;

  // camera origin → imu
  Eigen::Matrix<double, 3, 1> camera_origin;
  camera_origin << 0, 0, 0;

  Eigen::Matrix<double, 3, 3> imu2cameraMatrix_33d = imu2cameraMatrix_map_[camera_names_[0]].block<3, 3>(0, 0);
  Eigen::Matrix<double, 3, 1> imu2cameraMatrix_31d = imu2cameraMatrix_map_[camera_names_[0]].col(3);

  double camera2imu_origin_x = (imu2cameraMatrix_33d.inverse() * (camera_origin - imu2cameraMatrix_31d))(0);
  double camera2imu_origin_y = (imu2cameraMatrix_33d.inverse() * (camera_origin - imu2cameraMatrix_31d))(1);

  float camera_angle = 80;
  double angle_rad = (90 - camera_angle*0.5) * (CV_PI / 180.0);

  cv::Mat top_view_img(top_view_height, top_view_width, CV_8UC3, cv::Scalar(255, 255, 255));
  // cv::Mat front_view_img(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
  
  if (viz_switch) {
    cv::line(top_view_img, 
      cv::Point(top_view_width * 0.5, 0),
      cv::Point(top_view_width * 0.5, top_view_height), 
      cv::Scalar(125, 0, 125), 2);
    cv::line(top_view_img, 
      cv::Point(top_view_width * 0.5-80, 0),
      cv::Point(top_view_width * 0.5-80, top_view_height), 
      cv::Scalar(125, 0, 125), 2);
    cv::line(top_view_img, 
      cv::Point(top_view_width * 0.5+80, 0),
      cv::Point(top_view_width * 0.5+80, top_view_height), 
      cv::Scalar(125, 0, 125), 2);
    cv::circle(top_view_img, 
      cv::Point(0.5*top_view_width + 0.0f*top_view_width/x_range, top_view_height - 3.58f*top_view_height/y_range), 
      2, cv::Scalar(125, 0, 125), 25);

    double fov_x = x_range;
    double fov_y = std::tan(angle_rad) * fov_x;

    cv::Point start_point(
          (0.5 * top_view_width + camera2imu_origin_x * top_view_width / x_range),
          (top_view_height - camera2imu_origin_y * top_view_height / y_range));

    cv::Point end_point_plus(
      (0.5 * top_view_width + (camera2imu_origin_x + fov_x) * top_view_width / x_range), 
      (top_view_height - (camera2imu_origin_y + fov_y) * top_view_height / y_range));

    cv::Point end_point_minus(
      (0.5 * top_view_width + (camera2imu_origin_x - fov_x) * top_view_width / x_range), 
      (top_view_height - (camera2imu_origin_y + fov_y) * top_view_height / y_range));

    cv::line(top_view_img, start_point, end_point_plus, cv::Scalar(0, 0, 0), 1);
    cv::line(top_view_img, start_point, end_point_minus, cv::Scalar(0, 0, 0), 1);
  }

  box_roi_pcd_msgs_.clear();
  box_near_pcd_msgs_.clear();
  box_roi_pcd_msgs_erase.clear();

  anchor_width_vec.clear();
  anchor_length_vec.clear();
  anchor_height_vec.clear();

  // box_pcd_data = std::make_shared<PointCloud>();
  Eigen::Matrix<double, 3, 1>  projection_matrix_31d ;
  for (auto point : in_pcd_message->point()) {
    // if (point.y() >=7 || 6 <= point.z() || 0.0 >= point.z()) continue;
    if (0.0 >= point.z()) continue;

    Eigen::Matrix<double, 4, 1>  bp_projection_41d = Eigen::Matrix<double, 4, 1> ::Identity();
    bp_projection_41d << point.x(), point.y(), point.z(), 1;
    projection_matrix_31d = resultMatrix_map_[camera_names_[0]] * bp_projection_41d ;
    
    int box_id = 0;
    for(auto& box : in_box_message->perception_obstacle()){

      auto nomal_x = projection_matrix_31d(0)/std::abs(projection_matrix_31d(2));
      auto nomal_y = projection_matrix_31d(1)/std::abs(projection_matrix_31d(2));

      if(((box.bbox2d().xmin() <= nomal_x) && ( nomal_x <= box.bbox2d().xmax())) 
          && ((box.bbox2d().ymin() <= nomal_y) && ( nomal_y <= box.bbox2d().ymax()))){
      
        std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
        box_roi_pcd_msg_-> x = point.x();
        box_roi_pcd_msg_-> y = point.y();
        box_roi_pcd_msg_-> z = point.z();
        box_roi_pcd_msg_-> distance = std::sqrt(point.x()*point.x() + point.y()*point.y());
        box_roi_pcd_msg_-> id = box_id;
        box_roi_pcd_msg_-> label = static_cast<base::ObjectType>(box.type());
        box_roi_pcd_msg_-> sub_label = static_cast<base::ObjectSubType>(box.sub_type());

        // if(viz_switch){
        //   // cv::circle(front_view_img, cv::Point(nomal_x, nomal_y), 1, colors[0];, 1);
        //   cv::circle(top_view_img, 
        //       cv::Point(0.5*top_view_width + point.x()*top_view_width/x_range, top_view_height - point.y()*top_view_height/y_range), 
        //       1, colors[0], 1);
        // }
        box_roi_pcd_msgs_.push_back(std::move(box_roi_pcd_msg_));

        break;
      }
      box_id++;
    }
  }

  for (int i =0 ; i < in_box_message->perception_obstacle_size();i++){
    float near_point = 100.0;
    std::shared_ptr<PointIL> box_near_pcd_msg_ = std::make_shared<PointIL>();
    for (const auto& box_ : box_roi_pcd_msgs_ ){
      if (box_->id == i){
        if(box_->distance < near_point){
          near_point = box_->distance ;
          box_near_pcd_msg_-> x = box_->x;
          box_near_pcd_msg_-> y = box_->y;
          box_near_pcd_msg_-> z = box_->z;
          box_near_pcd_msg_-> distance = box_->distance;
          box_near_pcd_msg_-> id = box_->id;
          box_near_pcd_msg_-> label = box_->label;
          box_near_pcd_msg_-> sub_label = box_->sub_label;
        }
      }
    }
    if (near_point == 100.0){continue;}

    //가장 가까운 점을 구하고 일정 거리 이상의 점은 다 삭제하여 min max가 커지는 것을 방지함
    //occlusion_filter의 경우 추 후 실험을 통해 값을 변경해야하며, 지금은 5로 설정 되어있으며 conf 파일에서 수정 가능
    for (const auto& box_ : box_roi_pcd_msgs_){
      if (box_->id == i){
        if(box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msgs_erase.push_back(std::move(box_roi_pcd_msg_));
          if(viz_switch){
            cv::circle(top_view_img, 
                cv::Point(0.5*top_view_width + box_->x*top_view_width/x_range, top_view_height - box_->y*top_view_height/y_range), 
                1, colors[0], 1);
          }
        } else {
          if(viz_switch){
            cv::circle(top_view_img, 
                cv::Point(0.5*top_view_width + box_->x*top_view_width/x_range, top_view_height - box_->y*top_view_height/y_range), 
                1, colors[2], 1);
          }
        }
      }
    }

    double pcd_box_xmin = 100000;
    double pcd_box_ymin = 100000;
    double pcd_box_xmax = -100000;
    double pcd_box_ymax = -100000;
    for (const auto& box_ : box_roi_pcd_msgs_erase ){
      if (box_->id == i){
        if(box_->x < pcd_box_xmin){
          pcd_box_xmin = box_->x;
        }
        if(box_->y < pcd_box_ymin){
          pcd_box_ymin = box_->y;
        }
        if(box_->x > pcd_box_xmax){
          pcd_box_xmax = box_->x;
        }
        if(box_->y > pcd_box_ymax){
          pcd_box_ymax = box_->y;
        }
      }
    }

    double anchor_width = 1.0f;
    double anchor_length = 1.0f;
    double anchor_height = 1.0f;

    // float car_margin = 0.15;
    // float bicycle_margin = 0.1;
    // float pedestrian_margin = 0.06;
    // if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE)
    // {
    //   anchor_width = CAR_WIDTH - car_margin;
    //   anchor_length = CAR_LENGTH - car_margin;
    // }
    // else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE)
    // {
    //   anchor_width = PEDESTRIAN_WIDTH - bicycle_margin;
    //   anchor_length = PEDESTRIAN_LENGTH - bicycle_margin;
    // }
    // else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN)
    // {
    //   anchor_width = BICYCLE_WIDTH - pedestrian_margin;
    //   anchor_length = BICYCLE_LENGTH - pedestrian_margin;
    // }

    // 범위
    float offset = 6;

    double y = std::tan(angle_rad) * std::abs(box_near_pcd_msg_->x) + camera2imu_origin_y;
    double y_thd = y + offset;
    double percent_offset = (y_thd - box_near_pcd_msg_-> y) * anchor_length / offset;

    if (viz_switch) {
      // 임계영역 확인
      float trans_x0 = 0.5*top_view_width + (box_near_pcd_msg_-> x)*top_view_width/x_range;
      float trans_y0 = top_view_height - (y + camera2imu_origin_y)*top_view_height/y_range;
      float trans_y1 = top_view_height - (y + offset + camera2imu_origin_y)*top_view_height/y_range;

      // Fov
      cv::circle(top_view_img, cv::Point(trans_x0, trans_y0), 5, cv::Scalar(125, 0, 125), -1);
      // Thd
      cv::circle(top_view_img, cv::Point(trans_x0, trans_y1), 5, cv::Scalar(0, 255, 0), -1);
    }

    if((pcd_box_xmin <= 0 && pcd_box_xmax >= 0) || (pcd_box_xmin <= 0 && pcd_box_xmax >= -1.0) || (pcd_box_xmin <= 1.0 && pcd_box_xmax >= 0)) {
      if(pcd_box_ymin > 0 && pcd_box_ymax > 0) {
        box_near_pcd_msg_-> x = pcd_box_xmax - (pcd_box_xmax - pcd_box_xmin)/2;//anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
        // if(pcd_box_ymax < y_thd) {
        //   box_near_pcd_msg_-> x = pcd_box_xmax - (pcd_box_xmax - pcd_box_xmin)/2;//anchor_width/2;
        //   box_near_pcd_msg_-> y = pcd_box_ymin - anchor_length/2;  
        //   // cout << "touch" << endl;
        // }
        if (y_thd > y) {
          box_near_pcd_msg_-> x = pcd_box_xmax - (pcd_box_xmax - pcd_box_xmin)/2;
          box_near_pcd_msg_-> y -= percent_offset;
        }
      }
      else if(pcd_box_ymin <= 0 && pcd_box_ymax <= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmax - (pcd_box_xmax - pcd_box_xmin)/2;//anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
      }
    } 
    else if(pcd_box_xmin <= 0 && pcd_box_xmax <= 0) {
      if(pcd_box_ymin <= 0 && pcd_box_ymax <= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
      }
      else if(pcd_box_ymin <= 0 && pcd_box_ymax >= 0) {
        if(pcd_box_ymax - pcd_box_ymin < 1.0){
          box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
          box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
        }
        else{
          box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
          box_near_pcd_msg_-> y = pcd_box_ymax - (pcd_box_ymax - pcd_box_ymin)/2;
        }
      }
      else{
        // if(pcd_box_ymax < y_thd) {
        //   box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
        //   box_near_pcd_msg_-> y = pcd_box_ymin - anchor_length/2;  
        //   // cout << "left touch" << endl;
        // }
        if (y_thd > y) {
          box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
          box_near_pcd_msg_-> y -= percent_offset;
        }
      }
    } 
    
    else if(pcd_box_xmin >= 0 && pcd_box_xmax >= 0) {
      // box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
      // box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
      if(pcd_box_ymin <= 0 && pcd_box_ymax <= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
      }
      else if(pcd_box_ymin <= 0 && pcd_box_ymax >= 0) {
        if(pcd_box_ymax - pcd_box_ymin < 1.0){
          box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
          box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
        }
        else{
          box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
          // box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
          box_near_pcd_msg_-> y = pcd_box_ymax - (pcd_box_ymax - pcd_box_ymin)/2;
        }
      }
      else{
        box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
        // if(pcd_box_ymax < y_thd) {
        //   box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
        //   box_near_pcd_msg_-> y = pcd_box_ymin - anchor_length/2;  
        //   // cout << "right touch" << endl;
        // }
        if (y_thd > y) {
          box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
          box_near_pcd_msg_-> y -= percent_offset;
        }
      }
    } 

    if(viz_switch) {
      float trans_x = 0.5*top_view_width + (box_near_pcd_msg_-> x)*top_view_width/x_range;
      float trans_y = top_view_height - (box_near_pcd_msg_-> y)*top_view_height/y_range;

      // center point
      cv::circle(top_view_img, cv::Point(trans_x, trans_y), 2, cv::Scalar(0, 0, 0), 1);
    }

    box_near_pcd_msgs_.push_back(std::move(box_near_pcd_msg_));
    anchor_width_vec.push_back(anchor_width);
    anchor_length_vec.push_back(anchor_length);
    anchor_height_vec.push_back(anchor_height);
  }

  base::FramePtr bp_cam_fusion_frame(new base::Frame());
  int index = 0;
  for(auto& near_point_ : box_near_pcd_msgs_) {
    base::ObjectPtr obj(new base::Object);

    double anchor_width = anchor_width_vec[index];
    double anchor_length = anchor_length_vec[index];
    double anchor_height = anchor_height_vec[index];
    index++;

    obj->id = nearest_obstacle_id;
    obj->track_id = nearest_obstacle_id;
    nearest_obstacle_id++;
    if(nearest_obstacle_id > 9999) {
      nearest_obstacle_id = 1;
    }
    if(near_point_->label == base::ObjectType::VEHICLE){
      obj->type = base::ObjectType::UNKNOWN;
    }
    else{obj->type = near_point_->label;}
    obj->type_probs[static_cast<int>(obj->type)] = 1.0f;
    obj->sub_type = near_point_->sub_label;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = 1.0f;
    obj->distance = near_point_->distance;
    obj->center = Eigen::Vector3d(near_point_->x,near_point_->y,1.0);
    //hoseob start
    obj->center = pose * obj->center;

    double view_x0 = near_point_->x - anchor_width/2;
    double view_y0 = near_point_->y - anchor_length/2;
    double view_x1 = near_point_->x - anchor_width/2;
    double view_y1 = near_point_->y + anchor_length/2;
    double view_x2 = near_point_->x + anchor_width/2;
    double view_y2 = near_point_->y + anchor_length/2;
    double view_x3 = near_point_->x + anchor_width/2;
    double view_y3 = near_point_->y - anchor_length/2;
    Eigen::Vector3d point1 = Eigen::Vector3d(view_x0, view_y0, 1.0);
    Eigen::Vector3d point2 = Eigen::Vector3d(view_x1, view_y1, 1.0);
    Eigen::Vector3d point3 = Eigen::Vector3d(view_x2, view_y2, 1.0);
    Eigen::Vector3d point4 = Eigen::Vector3d(view_x3, view_y3, 1.0);
    point1 = pose * point1;
    point2 = pose * point2;
    point3 = pose * point3;
    point4 = pose * point4;

    obj->polygon.resize(4);
    obj->polygon[0].x = point1.x();
    obj->polygon[0].y = point1.y();
    obj->polygon[0].z = point1.z();
    obj->polygon[1].x = point2.x();
    obj->polygon[1].y = point2.y();
    obj->polygon[1].z = point2.z();
    obj->polygon[2].x = point3.x();
    obj->polygon[2].y = point3.y();
    obj->polygon[2].z = point3.z();
    obj->polygon[3].x = point4.x();
    obj->polygon[3].y = point4.y();
    obj->polygon[3].z = point4.z();
    //hoseob end

    obj->anchor_point = obj->center;

    Eigen::Vector3f velocity(0.0f, 0.0f, 0.0f);
    obj->velocity = velocity;

    // obj->center_uncertainty = Eigen::Matrix3f::Zero();
    // obj->velocity_uncertainty = Eigen::Matrix3f::Zero();
    Eigen::Matrix3d temp_matrix;
    temp_matrix.setIdentity();
    temp_matrix *= 0.75;
    obj->center_uncertainty = temp_matrix.cast<float>();
    obj->velocity_uncertainty = temp_matrix.cast<float>();
    Eigen::Vector3f direction(0.0f, 0.0f, 0.0f);

    base::PointD center_pointd;
    center_pointd.x = obj->center(0);
    center_pointd.y = obj->center(1);
    center_pointd.z = obj->center(2);
    if(hdmap_input_->GetNearestLaneDirection(center_pointd, &lane_direction)) {
      obj->direction[0] = lane_direction[0];
      obj->direction[1] = lane_direction[1];
      obj->direction[2] = lane_direction[2];
    } else {
      Eigen::Matrix<double, 3, 3> resultMatrix_33d = resultMatrix_map_[camera_names_[0]].block<3, 3>(0, 0);
      obj->direction[0] = static_cast<float>(resultMatrix_33d(0,0));
      obj->direction[1] = static_cast<float>(resultMatrix_33d(1,1));
      obj->direction[2] = 0;
    }
    obj->theta = static_cast<float>(atan2(obj->direction[1], obj->direction[0]));
    // AERROR << "theta1 : " << obj->track_id << "," << obj->theta*180/3.141592;
    obj->theta_variance = 0.0f;
    obj->confidence = 1.0f;

    obj->motion_state = base::MotionState::UNKNOWN;

    obj->size(0) = anchor_length;//1.0f;
    obj->size(1) = anchor_width;//1.0f;
    obj->size(2) = anchor_height;//1.0f;

    Eigen::Vector3d size_cuboid_standard;
    size_cuboid_standard = obj->size.cast<double>();
    obj->size_cuboid_standard = size_cuboid_standard.cast<float>();
    /////////////////////////////////////////////////////////////////
    bp_cam_fusion_frame->objects.push_back(obj);

    if(viz_switch){
      cv::rectangle(top_view_img, 
        cv::Point( 0.5*top_view_width + (view_x0)*top_view_width/x_range, top_view_height - (view_y0)*top_view_height/y_range),
        cv::Point( 0.5*top_view_width + (view_x2)*top_view_width/x_range, top_view_height - (view_y2)*top_view_height/y_range),
        cv::Scalar(0, 0, 0), 1);
    }
  }

  if(viz_switch){
    std::string img_time = std::to_string(Time::Now().ToNanosecond());
    
    // std::string front_view_name = sub_lidar_fusion_name + "Front View";
    std::string top_view_name = sub_lidar_fusion_name + "Top View";

    // cv::namedWindow(front_view_name, cv::WINDOW_NORMAL);
    // cv::resizeWindow(front_view_name, 1000, 600);
    // cv::imshow(front_view_name, front_view_img);
    // cv::waitKey(10);

    cv::namedWindow(top_view_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(top_view_name, 850, 1000);
    cv::imshow(top_view_name, top_view_img);
    cv::waitKey(10);
  }

  bp_cam_fusion_frame->timestamp = in_pcd_message->header().timestamp_sec();

  std::vector<base::ObjectPtr> bp_cam_fusion_objects;
  if (!bp_cam_fusion_perception_->Perceivebp_cam_fusion(bp_cam_fusion_frame,
                                   &bp_cam_fusion_objects)) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "RadarDetector Proc failed.";
    return true;
  }

  ++seq_num_;
  
  out_message->timestamp_ = in_pcd_message->header().timestamp_sec();//hoseob
  out_message->seq_num_ = seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_RECOGNITION;
  out_message->frame_.reset(new base::Frame());
  out_message->frame_->sensor2world_pose = pose;
  out_message->frame_->sensor_info = lidar_info_;
  out_message->frame_->objects = bp_cam_fusion_objects;
  out_message->sensor_id_ = sub_lidar_fusion_name;

  return true;
}

double SwmBpCamFusionComponent::RoiBoxLength(const double roi_box_x_min,
                                             const double roi_box_x_max,
                                             const double roi_box_y_min,
                                             const double roi_box_y_max,
                                             const int ind){   
  double roi_box_xmin = 100000;
  double roi_box_ymin = 100000;
  double roi_box_xmax = -100000;
  double roi_box_ymax = -100000;
  for (const auto& box_ : box_roi_pcd_msgs_erase ){
    if (box_->id == ind){
      if(box_->x >=  roi_box_x_min && box_->x <= roi_box_x_max && box_->y >=  roi_box_y_min && box_->y <=  roi_box_y_max ){
        if(box_->x < roi_box_xmin){
          roi_box_xmin = box_->x;
        }
        if(box_->y < roi_box_ymin){
          roi_box_ymin = box_->y;
        }
        if(box_->x > roi_box_xmax){
          roi_box_xmax = box_->x;
        }
        if(box_->y > roi_box_ymax){
          roi_box_ymax = box_->y;
        }
      }
    }
  }
  return roi_box_ymax - roi_box_ymin;
 }

bool SwmBpCamFusionComponent::InitAlgorithmPlugin() {

  std::string perception_method_ = "BpCamFusionObstaclePerception";
  std::string pipeline_name_ = "Frontbp_cam_fusionPipeline";

  bp_cam_fusion::BaseBpCamFusionObstaclePerception* bp_cam_fusion_perception =
      bp_cam_fusion::BaseBpCamFusionObstaclePerceptionRegisterer::GetInstanceByName(
          perception_method_);
  ACHECK(bp_cam_fusion_perception != nullptr)
      << "No bp_cam_fusion obstacle perception named: " << perception_method_;
  bp_cam_fusion_perception_.reset(bp_cam_fusion_perception);

  ACHECK(bp_cam_fusion_perception_->Init(pipeline_name_))
      << "Failed to init bp_cam_fusion perception.";
  AINFO << "Init algorithm plugin successfully.";

  for (const auto &camera_name : camera_names_) {
    // AERROR << camera_name ;
    base::BaseCameraModelPtr model;
    model = common::SensorManager::Instance()->GetUndistortCameraModel(camera_name);
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
    intrinsic_map_[camera_name] = intrinsic;
    AINFO << "#intrinsics of " << camera_name << ": "
          << intrinsic_map_[camera_name];
    Eigen::Matrix4d cam_extrinsic;

    std::string cam_name ;
    std::string lid_extrinsic_path ;
    if( camera_name == "am20_front_center_right_down"){
      cam_name = "cam_a_1";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_a_extrinsics.yaml";
    }
    else if( camera_name == "am20_front_right_rear"){
      cam_name = "cam_a_2";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_b_extrinsics.yaml";
    }
    else if( camera_name == "am20_rear_center_right"){
      cam_name = "cam_a_3";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_c_extrinsics.yaml";
    }
    else if( camera_name == "am20_front_left_rear"){
      cam_name = "cam_a_4";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_d_extrinsics.yaml";
    }

    LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + cam_name +
                       "_extrinsics.yaml",
                   &cam_extrinsic);
    extrinsic_map_[camera_name] = cam_extrinsic;

    Eigen::Matrix<double, 3, 4>  cam_extrinsic_34d;
    cam_extrinsic_34d = cam_extrinsic.inverse().block<3, 4>(0, 0);

    Eigen::Matrix4d lid_extrinsic;
    LoadExtrinsics(lid_extrinsic_path,
                   &lid_extrinsic);

    Eigen::Matrix<double, 4, 4>  lid_extrinsic_44d;
    lid_extrinsic_44d = lid_extrinsic.inverse().block<4, 4>(0, 0);

    Eigen::Matrix<double, 3, 4> imu2cameraMatrix;
    imu2cameraMatrix = cam_extrinsic_34d * lid_extrinsic_44d;
    imu2cameraMatrix_map_[camera_name] = imu2cameraMatrix;

    Eigen::Matrix<double, 3, 4> resultMatrix;
    resultMatrix = intrinsic.cast<double>() * cam_extrinsic_34d * lid_extrinsic_44d;
    resultMatrix_map_[camera_name] = resultMatrix;
  }

  lidar2world_trans_.Init("imu","velodyne128", "world", "imu");
  
  // AERROR << "InitAlgorithmPlugin out is ok";
  
  return true;
  }

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
