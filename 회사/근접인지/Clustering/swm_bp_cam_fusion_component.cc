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
#include "modules/perception/bp_cam_fusion/lib/lshape/lshaped_fitting.h"
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"

#define EGO_HALFWIDTH 1.87*0.5
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
#define MAX_GRADIENT_SIZE 50
#define CLUSTERING
// #define GRADIENT
// #define ORIGINAL

// #define USE_LSHAPE

namespace apollo {
namespace perception {
namespace onboard {

using namespace apollo::perception::bp_cam_fusion;
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

  //##test
  // AERROR << "";
  //##

  cv::Mat top_view_img(top_view_height, top_view_width, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat front_view_img(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));

  if (viz_switch) {
    // top view
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
    cv::Point ego_start_point(
      (0.5 * top_view_width + (-EGO_HALFWIDTH) * top_view_width / x_range),
      (top_view_height - 0 * top_view_height / y_range));
    cv::Point ego_end_point(
      (0.5 * top_view_width + EGO_HALFWIDTH * top_view_width / x_range),
      (top_view_height - 3.58f * top_view_height / y_range));
    cv::rectangle(top_view_img, ego_start_point,
                ego_end_point, cv::Scalar(125, 0, 125), -1);

    //##
    // float offset_range = 5;
    // cv::Scalar line_color(125, 0, 125);
    // int num_lines = 8; // Number of lines to draw

    // for (int i = 1; i < num_lines; ++i) {
    //     float offset = offset_range + i * 5;
    //     cv::Point start_point(0, top_view_height - offset * top_view_height / y_range);
    //     cv::Point end_point(top_view_width, top_view_height - offset * top_view_height / y_range);
    //     cv::line(top_view_img, start_point, end_point, line_color, 1);
    // }
    //##

    // double fov_x = x_range;
    // double fov_y = std::tan(angle_r_rad) * fov_x;

    // cv::Point start_point(
    //   (0.5 * top_view_width + camera2imu_origin_x * top_view_width / x_range),
    //   (top_view_height - camera2imu_origin_y * top_view_height / y_range));

    // cv::Point end_point_plus(
    //   (0.5 * top_view_width + (camera2imu_origin_x + fov_x) * top_view_width / x_range), 
    //   (top_view_height - (camera2imu_origin_y + fov_y) * top_view_height / y_range));

    // cv::Point end_point_minus(
    //   (0.5 * top_view_width + (camera2imu_origin_x - fov_x) * top_view_width / x_range), 
    //   (top_view_height - (camera2imu_origin_y + fov_y) * top_view_height / y_range));

    // cv::line(top_view_img, start_point, end_point_plus, cv::Scalar(0, 0, 0), 1);
    // cv::line(top_view_img, start_point, end_point_minus, cv::Scalar(0, 0, 0), 1);
  }

#ifdef USE_LSHAPE
  BoxesInfo.clear();
#endif // USE_LSHAPE
  box_roi_pcd_msgs_.clear();
  box_near_pcd_msgs_.clear();
  box_roi_pcd_msgs_erase.clear();
  box_roi_pcd_msgs_dummy.clear();
  box_roi_pcd_msgs_cvc.clear();
  box_checks.clear();

  anchor_width_vec.clear();
  anchor_length_vec.clear();
  anchor_height_vec.clear();

  int q = 0;
  for(auto& box_size : in_box_message->perception_obstacle()){
    if(box_size.bbox2d().xmax() - box_size.bbox2d().xmin() >= 1650) {
        continue;
      }
      mean_box_dis[(int)q]=0;
      q++;
  }

  // box_pcd_data = std::make_shared<PointCloud>();
  Eigen::Matrix<double, 3, 1>  projection_matrix_31d;
  for (auto point : in_pcd_message->point()) {
    // if (point.y() >=7 || 6 <= point.z() || 0.0 >= point.z()) continue;
    if (0.0 >= point.z()) continue;

    Eigen::Matrix<double, 4, 1>  bp_projection_41d = Eigen::Matrix<double, 4, 1> ::Identity();
    bp_projection_41d << point.x(), point.y(), point.z(), 1;
    projection_matrix_31d = resultMatrix_map_[camera_names_[0]] * bp_projection_41d ;

    auto nomal_x = projection_matrix_31d(0)/std::abs(projection_matrix_31d(2));
    auto nomal_y = projection_matrix_31d(1)/std::abs(projection_matrix_31d(2));

    std::shared_ptr<Box_Point_check> box_check_ = std::make_shared<Box_Point_check>();
    box_check_-> x = point.x();
    box_check_-> y = point.y();
    box_check_-> z = point.z();
    box_check_-> distance = std::sqrt(point.x()*point.x() + point.y()*point.y());
    // box_check_-> id.push_back(box_id);
    // box_check_-> label = static_cast<base::ObjectType>(box.type());
    // box_check_-> sub_label = static_cast<base::ObjectSubType>(box.sub_type());
    // box_check_->box_xlength = box.bbox2d().xmax() - box.bbox2d().xmin();
    // box_check_->box_ylength = box.bbox2d().ymax() - box.bbox2d().ymin();
      
    
    int box_id = 0;
    for(auto& box : in_box_message->perception_obstacle()) {
      if(camera_names_[0] == "am20_front_center_right_down" && 
          (box.bbox2d().xmax() - box.bbox2d().xmin()) >= 1550) {
        continue;
      }

      if(((box.bbox2d().xmin() <= nomal_x) && ( nomal_x <= box.bbox2d().xmax())) 
          && ((box.bbox2d().ymin() <= nomal_y) && ( nomal_y <= box.bbox2d().ymax()))) {

        // if(viz_switch){
        //   cv::circle(front_view_img, cv::Point(nomal_x, nomal_y), 3, colors[0], -1);
        // }

        box_check_-> id.push_back(box_id);
        box_check_-> label.push_back(static_cast<base::ObjectType>(box.type()));
        box_check_-> sub_label.push_back(static_cast<base::ObjectSubType>(box.sub_type()));
        box_check_->box_xlength.push_back(box.bbox2d().xmax() - box.bbox2d().xmin());
        box_check_->box_ylength.push_back(box.bbox2d().ymax() - box.bbox2d().ymin());
        mean_box_dis[box_id] = (mean_box_dis[box_id] + box_check_-> distance)/2;

        box_checks.push_back(box_check_);  
      }
      box_id++;

      int exclude_box_id = 0;
      for(auto& exclude_box : in_box_message->perception_obstacle()) {
        if(camera_names_[0] == "am20_front_center_right_down" && 
          (exclude_box.bbox2d().xmax() - exclude_box.bbox2d().xmin()) >= 1550) {
        continue;
      }
        if(box_id == exclude_box_id) {
          continue;
        }
      if(((box.bbox2d().xmin() >= exclude_box.bbox2d().xmin()) && 
              (box.bbox2d().xmax() <= exclude_box.bbox2d().xmax())) &&
              ((box.bbox2d().ymin() >= exclude_box.bbox2d().ymin()) && 
              ( box.bbox2d().ymax() <= exclude_box.bbox2d().ymax()))) {
        over_box[box_id] = exclude_box_id;
            }
      exclude_box_id++;



    }
    // box_checks.push_back(box_check_);   

  }
  }

  for(auto& check_ : box_checks) {
    if(check_->id.size() == 0){
      continue;
    }
    else if(check_->id.size() == 1){
      std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
      box_roi_pcd_msg_-> x = check_-> x;
      box_roi_pcd_msg_-> y = check_-> y;
      box_roi_pcd_msg_-> z = check_-> z;
      box_roi_pcd_msg_-> distance = check_-> distance;
      box_roi_pcd_msg_-> id = check_-> id[0];
      box_roi_pcd_msg_-> label = static_cast<base::ObjectType>(check_->label[0]);
      box_roi_pcd_msg_-> sub_label = static_cast<base::ObjectSubType>(check_->sub_label[0]);
      box_roi_pcd_msg_-> box_xlength = check_->box_xlength[0];
      box_roi_pcd_msg_-> box_ylength = check_->box_ylength[0];
      box_roi_pcd_msgs_.push_back(std::move(box_roi_pcd_msg_));
    }
  }

  //최근접점을 찾는 부분
  for (int i =0 ; i < in_box_message->perception_obstacle_size();i++){  
    const auto& box = in_box_message->perception_obstacle(i);
    if (viz_switch) {
      // rectangle
      if(box.bbox2d().xmax() - box.bbox2d().xmin() < 1550) {
        cv::rectangle(front_view_img, cv::Point(box.bbox2d().xmin(), box.bbox2d().ymin()),
                    cv::Point(box.bbox2d().xmax(), box.bbox2d().ymax()), cv::Scalar(0, 0, 0), 2);
      }
    }
#ifdef CLUSTERING
    int box_id = 0;
    base::ObjectType label = base::ObjectType::UNKNOWN;
    base::ObjectSubType sub_label = base::ObjectSubType::UNKNOWN;
    double box_xlength = 0;
    double box_ylength = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_point(new pcl::PointCloud<pcl::PointXYZI>);

    double cluster_xmin = std::numeric_limits<double>::max();    // 최대값
    double cluster_ymin = std::numeric_limits<double>::max();    // 최대값
    double cluster_xmax = std::numeric_limits<double>::min();    // 최소값
    double cluster_ymax = std::numeric_limits<double>::min();    // 최소값

    for (const auto& box_ : box_roi_pcd_msgs_ ){
      if (box_->id == i){
        pcl::PointXYZI new_point;
        new_point.x = box_->x;
        new_point.y = box_->y;
        new_point.z = box_->z;
        new_point.intensity = 100;
        cluster_point->push_back(new_point);

        box_id = box_->id;
        label = box_->label;
        sub_label = box_->sub_label;
        box_xlength = box_->box_xlength;
        box_ylength = box_->box_ylength;

        if(box_->x < cluster_xmin){
          cluster_xmin = box_->x;
        }
        if(box_->y < cluster_ymin){
          cluster_ymin = box_->y;
        }
        if(box_->x > cluster_xmax){
          cluster_xmax = box_->x;
        }
        if(box_->y > cluster_ymax){
          cluster_ymax = box_->y;
        }
      }
    }

    if(cluster_point->size() > 1) {
      //###
      vector<float> param(3,0);
      param[0] = 361;  // 고도각 

      // if (cluster_ymax - cluster_ymin < 5) {
      //   param[1] = 0.7;
      // }

      if (static_cast<base::ObjectSubType>(box.sub_type()) == base::ObjectSubType::BUS) {  // BUS
        param[1] = 3; // 거리
      }
      else if (static_cast<base::ObjectSubType>(box.sub_type()) == base::ObjectSubType::CAR
        || static_cast<base::ObjectSubType>(box.sub_type()) == base::ObjectSubType::TRUCK
        || static_cast<base::ObjectSubType>(box.sub_type()) == base::ObjectSubType::VAN) {
        param[1] = 1.8;
      }
      // else if (static_cast<base::ObjectSubType>(box.sub_type()) == base::ObjectSubType::TRUCK) {
      //   param[1] = 1.8;
      // }
      else {
        param[1] = 1;
      }

      param[2] = 361;  // 방위각 

      CVC Cluster(param);

      std::vector<PointAPR> capr;
      Cluster.calculateAPR(*cluster_point, capr);

      //##test
      // cout << "Object: " << i << endl;
      // cout << "Max Polar: " << Cluster.max_polar()* 180.0 / M_PI 
      //      << " Min Polar: " << Cluster.min_polar() * 180.0 / M_PI << endl;

      // cout << "Max Range: " << Cluster.max_range()
      //      << " Min Range: " << Cluster.min_range()<< endl;

      // cout << "Max Azimute: " << Cluster.max_azimuth()* 180.0 / M_PI 
      //      << " Min Azimute: " << Cluster.min_azimuth() * 180.0 / M_PI << endl;

      // cout << endl;

      [[maybe_unused]] double range = Cluster.max_range() - Cluster.min_range();
      [[maybe_unused]] int num_points = capr.size();
      // double range = Cluster.max_range() - Cluster.min_range();
      // int num_points = capr.size();

      // int num_points = box_roi_pcd_msgs_.size();
      //##

      std::unordered_map<int, Voxel> hash_table;
      Cluster.build_hash_table(capr, hash_table);

      vector<int> cluster_indices;	
      cluster_indices = Cluster.cluster(hash_table, capr);

      vector<int> cluster_id; // ex) {5, 4, 3, 2, 1}
      Cluster.most_frequent_value(cluster_indices, cluster_id);

      //##
      // 각 i에 대한 클러스터링 결과를 저장할 데이터 구조 생성
      std::vector<PointAPR> clustered_points;
      std::vector<int> clustered_ids;

      // 각 클러스터에 대한 정보를 저장
      for (int cluster_idx : cluster_id) {
          std::vector<PointAPR> cluster_points;
          int cluster_id = cluster_idx;

          for (size_t j = 0; j < cluster_indices.size(); j++) {
              if (cluster_indices[j] == cluster_idx) {
                  cluster_points.push_back(capr[j]);
              }
          }

          // 각 클러스터에 대한 정보를 저장
          clustered_points.insert(clustered_points.end(), cluster_points.begin(), cluster_points.end());
          clustered_ids.push_back(cluster_id);

          // cout << "Object: " << i << endl;
          // cout << "Cluster ID: " << cluster_id << ", Number of Points: " << cluster_points.size() << endl;
          // cout << endl;
      }

      if (viz_switch) {
        cv::rectangle(top_view_img, 
          cv::Point( 0.5*top_view_width + (cluster_xmin)*top_view_width/x_range, top_view_height - (cluster_ymin)*top_view_height/y_range),
          cv::Point( 0.5*top_view_width + (cluster_xmax)*top_view_width/x_range, top_view_height - (cluster_ymax)*top_view_height/y_range),
          cv::Scalar(0, 128, 0), 2);

        std::string text = std::to_string(i);
        cv::putText(top_view_img, text, 
          cv::Point(0.5*top_view_width + (cluster_xmin + cluster_xmax)*0.5*top_view_width/x_range-10, top_view_height - (cluster_ymin)*top_view_height/y_range+20),
          cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(125, 0, 125), 2);

        // // Create text strings
        // std::string text1 = "Objects: " + std::to_string(i) + "  [" + std::to_string(num_points) + "]  [" +
        //  ObjectSubTypeToString(static_cast<base::ObjectSubType>(box.sub_type())) + "]";
        // std::string cluster_text = "cluster: ";

        // std::string text2 = "range: " + std::to_string(range) + 
        //   "    r_intervals: " + std::to_string(Cluster.length());
        //   // "    box_height: " + std::to_string(cluster_ymax - cluster_ymin);
        //   // "    azimuth: " + std::to_string( (Cluster.max_azimuth() - Cluster.min_azimuth())* 180.0 / M_PI ) +
        //   // "    a_intervals: " + std::to_string(Cluster.height()) +
        //   // "    polar: " + std::to_string( (Cluster.max_polar() - Cluster.min_polar())* 180.0 / M_PI );

        // for (std::size_t j = 0; j < cluster_id.size(); ++j) {
        //     cluster_text += std::to_string(cluster_id[j]) + " ";

        //     // 해당 클러스터에 속하는 포인트들의 개수를 가져옴
        //     int points_in_cluster = 0;
        //     for (size_t k = 0; k < cluster_indices.size(); ++k) {
        //         if (cluster_indices[k] == cluster_id[j]) {
        //             ++points_in_cluster;
        //         }
        //     }

        //     // 포인트 개수도 텍스트로 추가
        //     cluster_text += "(" + std::to_string(points_in_cluster) + ") ";
        // }

        // // Calculate text positions based on 'i'
        // int text_y = top_view_height - 100 - i * 100;  // Adjust the value (20) for desired spacing
        // int text2_y = top_view_height - 50 - i * 100;

        // // Draw text on the image
        // cv::putText(top_view_img, text1, cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        // cv::putText(top_view_img, text2, cv::Point(10, text2_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        // // Draw cluster_id text on the image
        // cv::putText(top_view_img, cluster_text, cv::Point(550, text_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        ///////////////////////////////////////////////////

        for (size_t k = 0; k < cluster_indices.size(); ++k) {
          Eigen::Matrix<double, 4, 1>  bp_projection_41d = Eigen::Matrix<double, 4, 1> ::Identity();
          bp_projection_41d << cluster_point->points[k].x, cluster_point->points[k].y, cluster_point->points[k].z, 1;
          projection_matrix_31d = resultMatrix_map_[camera_names_[0]] * bp_projection_41d ;

          auto nomal_x = projection_matrix_31d(0)/std::abs(projection_matrix_31d(2));
          auto nomal_y = projection_matrix_31d(1)/std::abs(projection_matrix_31d(2));

          int clusterIndex = cluster_indices[k];
          int colorIndex = clusterIndex - 1;  // Adjust index to match the colors array
          
          if (colorIndex >= 0 && colorIndex < static_cast<int>(colors.size())) {
              cv::Point circleCenter(
                  0.5 * top_view_width + cluster_point->points[k].x * top_view_width / x_range,
                  top_view_height - cluster_point->points[k].y * top_view_height / y_range
              );
              
              cv::circle(top_view_img, circleCenter, 5, colors[colorIndex], -1);
              cv::circle(front_view_img, cv::Point(nomal_x, nomal_y), 3, colors[colorIndex], -1);
          }

          else {
              cv::Point circleCenter(
                  0.5 * top_view_width + cluster_point->points[k].x * top_view_width / x_range,
                  top_view_height - cluster_point->points[k].y * top_view_height / y_range
              );
              
              cv::circle(top_view_img, circleCenter, 3, cv::Scalar(0, 0, 0), -1);
              cv::circle(front_view_img, cv::Point(nomal_x, nomal_y), 3, cv::Scalar(0, 0, 0), -1);
          }
        }

      }
      //##
#if 0
      //점이 가장 많은 클러스터 찾기
      int count = 0;
      int max_cluster_id = 0;
      for(int j = 0; j < (int)cluster_id.size(); ++j){
        int o_count = 0;
        for(int k =0; k<(int)cluster_indices.size(); ++k){
          if(cluster_indices[k] == cluster_id[j]){
            o_count++;
          }
        }
        if(o_count > count) {
          count = o_count;
          max_cluster_id = cluster_id[j];
        }
      }
#else
      // std::map<int, float> cluster_id_z;
      // float maxz = 0;
      // int max_cluster_id = 0;
      // for(int j = 0; j < (int)cluster_id.size(); ++j){

      //   for(int k =0; k<(int)cluster_indices.size(); ++k){
      //     if(cluster_point->points[k].z > maxz){
      //       maxz = cluster_point->points[k].z;
      //       max_cluster_id = cluster_id[j];
      //     }
      //   }

      //   cluster_id_z[max_cluster_id] = maxz;
      // }

      //###
      std::map<int, float> cluster_id_z; // 클러스터 ID와 제곱합의 평균을 저장할 맵
      std::map<int, int> cluster_id_point_count; // 클러스터 ID와 포인트 개수를 저장할 맵

      // ex) cluster_id = {5, 4, 3, 2, 1}
      // ex) cluster_indices = {1, 2, 1, 2, 1, 1, ..., 5, 4, 2}
      for (int j = 0; j < static_cast<int>(cluster_id.size()); ++j) {

          float squared_diff_sum = 0.0f;
          int num_points = 0;

          for (int k = 0; k < static_cast<int>(cluster_indices.size()); ++k) {

              if (cluster_indices[k] == cluster_id[j]) {
                  // 반복문 시작
                  // float diff = cluster_point->points[k].z - PEDESTRIAN_HEIGHT * 0.5;
                  // squared_diff_sum += diff * diff;

                  squared_diff_sum += cluster_point->points[k].z;

                  ++num_points;
              }
          }

          if (num_points > 0) {
              float mean_squared_diff = squared_diff_sum / num_points;
              cluster_id_z[cluster_id[j]] = mean_squared_diff;
              cluster_id_point_count[cluster_id[j]] = num_points;
          }
      }

      // 가장 작은 제곱합의 평균과 해당 클러스터의 포인트 개수를 출력
      int min_diff_cluster_id = -1;
      float min_diff_value = std::numeric_limits<float>::max();
      int min_diff_point_count = 0;

      // ex) cluster_id_z = { {5:제곱합}, {4:제곱합}, {1:제곱합}, {2:제곱합}, {1:제곱합} }
      // ex) cluster_id_point_count = { {5:포인트수}, {4:포인트수}, {3:포인트수}, {2:포인트수}, {1:포인트수} } 
      for (const auto& entry : cluster_id_z) {
          if (entry.second < min_diff_value) {
              min_diff_value = entry.second;

              min_diff_cluster_id = entry.first;
              min_diff_point_count = cluster_id_point_count[entry.first];
          }
      }

      // cout << "Objetct: " << i << endl;
      // if (min_diff_cluster_id != -1) {
      //     std::cout << "Cluster id: " << min_diff_cluster_id 
      //               << " [" << min_diff_point_count << "] "  
      //               << min_diff_value << std::endl;
      //     // 나머지 클러스터 정보 출력
      //     for (const auto& entry : cluster_id_z) {
      //         if (entry.first != min_diff_cluster_id) {
      //             std::cout << "cluster id: " << entry.first 
      //                       << " [" << cluster_id_point_count[entry.first] << "] " 
      //                       << entry.second << std::endl;  // 평균오차제곱합
      //         }
      //     }
      // } else {
      //     std::cout << "No valid cluster found." << std::endl;
      // }
      // cout << endl;

      int max_cluster_id = min_diff_cluster_id;

      if (viz_switch) {
        std::string object_text = "Object: " + std::to_string(i)+ " [" +
         ObjectSubTypeToString(static_cast<base::ObjectSubType>(box.sub_type())) + "]";
        std::string cluster_text = std::to_string(min_diff_cluster_id) + " [" + std::to_string(min_diff_point_count) + "] " + std::to_string(min_diff_value);
        
        if (min_diff_cluster_id != -1) {
          // ex) cluster_id_point_count = { {5:포인트수}, {4:포인트수}, {3:포인트수}, {2:포인트수}, {1:포인트수} } 
          // ex) entry = {n:제곱합}
          for (const auto& entry : cluster_id_z) {
              if (entry.first != min_diff_cluster_id) {
                  cluster_text += " | " + std::to_string(entry.first) + " [" + std::to_string(cluster_id_point_count[entry.first]) + "] " + std::to_string(entry.second);
              }
          }
        }
        
        int object_y = top_view_height - 100 - i * 100; 
        int cluster_y = top_view_height - 50 - i * 100;

        // // Draw text on the image
        cv::putText(top_view_img, object_text, cv::Point(10, object_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
        cv::putText(top_view_img, cluster_text, cv::Point(10, cluster_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
      }

      //##

      /*
      float similar_z = 10000;
      for(auto& pair : cluster_id_z){
        if (std::abs(pair.second - labelz) <= similar_z){
          maxz = pair.second;
          max_cluster_id = pair.first;
        }
      }
      */
#endif
      for(int k =0; k<(int)cluster_indices.size(); ++k){
        if(cluster_indices[k] == max_cluster_id){
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = cluster_point->points[k].x;
          box_roi_pcd_msg_-> y = cluster_point->points[k].y;
          box_roi_pcd_msg_-> z = cluster_point->points[k].z;
          box_roi_pcd_msg_-> distance = 
              std::sqrt(cluster_point->points[k].x*cluster_point->points[k].x + cluster_point->points[k].y*cluster_point->points[k].y);
          box_roi_pcd_msg_-> id = box_id;
          box_roi_pcd_msg_-> label = label;
          box_roi_pcd_msg_-> sub_label = sub_label;
          box_roi_pcd_msg_-> box_xlength = box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_ylength;
          box_roi_pcd_msgs_cvc.push_back(std::move(box_roi_pcd_msg_));
          //## 클러스터링 선택O
          if(viz_switch){
            cv::circle(top_view_img, 
                cv::Point(0.5*top_view_width + cluster_point->points[k].x*top_view_width/x_range, 
                    top_view_height - cluster_point->points[k].y*top_view_height/y_range), 
                5, cv::Scalar(0, 125, 0), -1);
          }
        } 
      }
    }

    float near_point = 100.0;
    std::shared_ptr<PointIL> box_near_pcd_msg_ = std::make_shared<PointIL>();
    // for (const auto& box_ : box_roi_pcd_msgs_ ){
    for (const auto& box_ : box_roi_pcd_msgs_cvc ){
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
          box_near_pcd_msg_-> box_xlength = box_->box_xlength;
          box_near_pcd_msg_-> box_ylength = box_->box_ylength;
        }
      }
    }
    if (near_point == 100.0){continue;}

    //가장 가까운 점을 구하고 일정 거리 이상의 점은 다 삭제하여 min max가 커지는 것을 방지함
    //occlusion_filter의 경우 추 후 실험을 통해 값을 변경해야하며, 지금은 5로 설정 되어있으며 conf 파일에서 수정 가능
    // for (const auto& box_ : box_roi_pcd_msgs_){
    for (const auto& box_ : box_roi_pcd_msgs_cvc){
      if (box_->id == i){
        if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
          occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
        }
        else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
          occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
        }
        else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
          occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
        }
        // if(box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
        if(box_->distance > (box_near_pcd_msg_->distance - occlusion_filter/4) &&
            box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msg_-> box_xlength = box_->box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_->box_ylength;
          box_roi_pcd_msgs_erase.push_back(std::move(box_roi_pcd_msg_));
          if(viz_switch){
            // cv::circle(top_view_img, 
            //     cv::Point(0.5*top_view_width + box_->x*top_view_width/x_range, top_view_height - box_->y*top_view_height/y_range), 
            //     1, colors[0], 1);
          }
        } else {
          if(viz_switch){
            // cv::circle(top_view_img, 
            //     cv::Point(0.5*top_view_width + box_->x*top_view_width/x_range, top_view_height - box_->y*top_view_height/y_range), 
            //     1, colors[2], 1);
          }
        }
      }
    }
    //다른 박스에 포함 및 겹치는 박스일 경우 거리가 가까운 것들만 최종 처리할 수 있도록 함
    for (const auto& box_ : box_roi_pcd_msgs_dummy){
      if (box_->id == i){
        // if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
        //   // occlusion_filter = CAR_LENGTH;
        //   occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
        // }
        // else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
        //   // occlusion_filter = PEDESTRIAN_LENGTH;
        //   occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
        // }
        // else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
        //   // occlusion_filter = BICYCLE_WIDTH;
        //   occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
        // }
        // if(box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
        if(box_->distance > (box_near_pcd_msg_->distance - occlusion_filter/4) &&
            box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msg_-> box_xlength = box_->box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_->box_ylength;
          box_roi_pcd_msgs_erase.push_back(std::move(box_roi_pcd_msg_));
          if(viz_switch){
            // cv::circle(top_view_img, 
            //     cv::Point(0.5*top_view_width + box_->x*top_view_width/x_range, top_view_height - box_->y*top_view_height/y_range), 
            //     1, colors[0], 1);
          }
        } else {
          if(viz_switch){
            // cv::circle(top_view_img, 
            //     cv::Point(0.5*top_view_width + box_->x*top_view_width/x_range, top_view_height - box_->y*top_view_height/y_range), 
            //     1, colors[2], 1);
          }
        }
      }
    }
#elif defined(GRADIENT)
    double gradient_distance;
    float near_point = 100.0;
    std::shared_ptr<PointIL> box_near_pcd_msg_ = std::make_shared<PointIL>();
    for (const auto& box_ : box_roi_pcd_msgs_){
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
          box_near_pcd_msg_-> box_xlength = box_->box_xlength;
          box_near_pcd_msg_-> box_ylength = box_->box_ylength;
        }
      }
    }
    if (near_point == 100.0){continue;}

    double gradient = 0;
    if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::CAR) {
      gradient = car_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::VAN) {
      gradient = van_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::TRUCK) {
      gradient = truck_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::BUS) {
      gradient = bus_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::CYCLIST) {
      gradient = cyclist_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::MOTORCYCLIST) {
      gradient = motorcyclist_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::TRICYCLIST) {
      gradient = tricyclist_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::PEDESTRIAN) {
      gradient = pedestrian_gradient;
    } else if(box_near_pcd_msg_-> sub_label == base::ObjectSubType::TRAFFICCONE) {
      gradient = trafficon_gradient;
    }
    // double gradient_distance = gradient * box_near_pcd_msg_-> box_ylength;
    // gradient_distance = gradient * 1204.146818/ box_near_pcd_msg_-> box_ylength;
    gradient_distance = gradient * f_x/ box_near_pcd_msg_-> box_ylength;
    if(gradient_distance == 0) {
      gradient_distance = box_near_pcd_msg_-> distance;
    }

    //가장 가까운 점을 구하고 일정 거리 이상의 점은 다 삭제하여 min max가 커지는 것을 방지함
    //occlusion_filter의 경우 추 후 실험을 통해 값을 변경해야하며, 지금은 5로 설정 되어있으며 conf 파일에서 수정 가능
    // for (const auto& box_ : box_roi_pcd_msgs_){
    for (const auto& box_ : box_roi_pcd_msgs_){
      if (box_->id == i){
        if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
          // occlusion_filter = CAR_LENGTH;
          occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
        }
        else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
          // occlusion_filter = PEDESTRIAN_LENGTH;
          occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
        }
        else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
          // occlusion_filter = BICYCLE_WIDTH;
          occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
        }
        // if(box_->distance < (gradient_distance + occlusion_filter)) {
        if(box_->distance > (gradient_distance - occlusion_filter/4) &&
            box_->distance < (gradient_distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msg_-> box_xlength = box_->box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_->box_ylength;
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
    //다른 박스에 포함 및 겹치는 박스일 경우 거리가 가까운 것들만 최종 처리할 수 있도록 함
    for (const auto& box_ : box_roi_pcd_msgs_dummy){
      if (box_->id == i){
        if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
          // occlusion_filter = CAR_LENGTH;
          occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
        }
        else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
          // occlusion_filter = PEDESTRIAN_LENGTH;
          occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
        }
        else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
          // occlusion_filter = BICYCLE_WIDTH;
          occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
        }
        // if(box_->distance < (gradient_distance + occlusion_filter)) {
        if(box_->distance > (gradient_distance - occlusion_filter/4) &&
            box_->distance < (gradient_distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msg_-> box_xlength = box_->box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_->box_ylength;
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

    // float near_point = 100.0;
    near_point = 100.0;
    // std::shared_ptr<PointIL> box_near_pcd_msg_ = std::make_shared<PointIL>();
    // for (const auto& box_ : box_roi_pcd_msgs_ ){
    for (const auto& box_ : box_roi_pcd_msgs_erase){
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
          box_near_pcd_msg_-> box_xlength = box_->box_xlength;
          box_near_pcd_msg_-> box_ylength = box_->box_ylength;
        }
      }
    }
    if (near_point == 100.0){
#ifdef GRADIENT      
      cv::line(top_view_img, 
          cv::Point(0.5*top_view_width + (box_near_pcd_msg_->x - 1)*top_view_width/x_range, 
              top_view_height - gradient_distance*top_view_height/y_range),
          cv::Point(0.5*top_view_width + (box_near_pcd_msg_->x + 1)*top_view_width/x_range, 
              top_view_height - gradient_distance*top_view_height/y_range), 
          cv::Scalar(125, 0, 125), 2);
#endif          
      continue;
    }
#elif defined(ORIGINAL)
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
          box_near_pcd_msg_-> box_xlength = box_->box_xlength;
          box_near_pcd_msg_-> box_ylength = box_->box_ylength;
        }
      }
    }
    if (near_point == 100.0){continue;}

    //가장 가까운 점을 구하고 일정 거리 이상의 점은 다 삭제하여 min max가 커지는 것을 방지함
    //occlusion_filter의 경우 추 후 실험을 통해 값을 변경해야하며, 지금은 5로 설정 되어있으며 conf 파일에서 수정 가능
    for (const auto& box_ : box_roi_pcd_msgs_){
      if (box_->id == i){
        if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
          // occlusion_filter = CAR_LENGTH;
          occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
        }
        else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
          // occlusion_filter = PEDESTRIAN_LENGTH;
          occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
        }
        else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
          // occlusion_filter = BICYCLE_WIDTH;
          occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
        }
        // if(box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
        if(box_->distance > (box_near_pcd_msg_->distance - occlusion_filter/4) &&
            box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msg_-> box_xlength = box_->box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_->box_ylength;
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
    //다른 박스에 포함 및 겹치는 박스일 경우 거리가 가까운 것들만 최종 처리할 수 있도록 함
    for (const auto& box_ : box_roi_pcd_msgs_dummy){
      if (box_->id == i){
        if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
          // occlusion_filter = CAR_LENGTH;
          occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
        }
        else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
          // occlusion_filter = PEDESTRIAN_LENGTH;
          occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
        }
        else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
          // occlusion_filter = BICYCLE_WIDTH;
          occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
        }
        // if(box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
        if(box_->distance > (box_near_pcd_msg_->distance - occlusion_filter/4) &&
            box_->distance < (box_near_pcd_msg_->distance + occlusion_filter)) {
          std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
          box_roi_pcd_msg_-> x = box_->x;
          box_roi_pcd_msg_-> y = box_->y;
          box_roi_pcd_msg_-> z = box_->z;
          box_roi_pcd_msg_-> distance = box_->distance;
          box_roi_pcd_msg_-> id = box_->id;
          box_roi_pcd_msg_-> label = box_->label;
          box_roi_pcd_msg_-> sub_label = box_->sub_label;
          box_roi_pcd_msg_-> box_xlength = box_->box_xlength;
          box_roi_pcd_msg_-> box_ylength = box_->box_ylength;
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
#endif
#ifdef USE_LSHAPE
    // start Lshape.
    bool lshape_state_ = false;
    std::shared_ptr<BoxInfo> boxInfo = std::make_shared<BoxInfo>();

    for (const auto& box_ : box_roi_pcd_msgs_erase) {
      if (box_->id==i) {
        boxInfo->id = box_->id;
        boxInfo->points.push_back(cv::Point2f(box_->x, box_->y));
      }
    }

    LShapedFIT lshaped;
    cv::RotatedRect box_result = lshaped.FitBox(&boxInfo->points, lshape_state_);
    cv::Point2f box_points[4];
    box_result.points(box_points);

    for (const auto& box_ : box_roi_pcd_msgs_erase) {
      if (box_->id==i) {
        boxInfo->box_center = box_result.center;
        boxInfo->box_point1 = box_points[0];
        boxInfo->box_point2 = box_points[1];
        boxInfo->box_point3 = box_points[2];
        boxInfo->box_point4 = box_points[3];
        boxInfo->lshape_state = lshape_state_;
      }
    }
    BoxesInfo.push_back(boxInfo);
    // end Lshape.
#endif // USE_LSHAPE

    //박스의 min x, y, max x, y를 구하는 부분
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

    //default anchor size 설정
    double anchor_width = 1.0f;
    double anchor_length = 1.0f;
    double anchor_height = 1.0f;

    //type에 따른 anchor size 적용
    // float car_margin = 0.15;
    float bicycle_margin = 0.1;
    float pedestrian_margin = 0.06;
    if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE)
    {
      // anchor_width = CAR_WIDTH - car_margin;
      // anchor_length = CAR_LENGTH - car_margin;
      anchor_width = 1.0f;
      anchor_length = 1.0f;
    }
    else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE)
    {
      anchor_width = PEDESTRIAN_WIDTH - bicycle_margin;
      anchor_length = PEDESTRIAN_LENGTH - bicycle_margin;
    }
    else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN)
    {
      anchor_width = BICYCLE_WIDTH - pedestrian_margin;
      anchor_length = BICYCLE_LENGTH - pedestrian_margin;
    }
    //전방     
    if(camera_names_[0] == "am20_front_center_right_down") {
      float offset = anchor_length;
      y_r = std::tan(angle_r_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
      y_r_thd = y_r + offset;

      y_l = std::tan(angle_l_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
      y_l_thd = y_l + offset;

      // if (viz_switch) {
      //   // 임계영역 확인
      //   float trans_x0 = 0.5*top_view_width + (box_near_pcd_msg_-> x)*top_view_width/x_range;
      //   float trans_y0 = top_view_height - (y)*top_view_height/y_range;
      //   float trans_y1 = top_view_height - (y_thd)*top_view_height/y_range;
      //   // Fov
      //   cv::circle(top_view_img, cv::Point(trans_x0, trans_y0), 5, cv::Scalar(125, 0, 125), -1);
      //   // Thd
      //   cv::circle(top_view_img, cv::Point(trans_x0, trans_y1), 5, cv::Scalar(0, 255, 0), -1);
      // }

      //obstacle의 lidar min/max좌표가 ego width의 안쪽에 있을 경우
      //ego를 통과해서 지날 수 없기 때문에 앞쪽 혹은 뒤쪽에만 존재
      if((pcd_box_xmin <= 0 && pcd_box_xmax >= 0) ||
          (pcd_box_xmin >= -EGO_HALFWIDTH && pcd_box_xmin <= EGO_HALFWIDTH) || 
          (pcd_box_xmax >= -EGO_HALFWIDTH && pcd_box_xmax <= EGO_HALFWIDTH)) {
        box_near_pcd_msg_-> x = pcd_box_xmax - (pcd_box_xmax - pcd_box_xmin)/2;
        box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
      }
      else if(pcd_box_xmin <= 0 && pcd_box_xmax <= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
        if(box_near_pcd_msg_-> y <= y_l_thd) {
          if((pcd_box_ymax - pcd_box_ymin) < anchor_length) {
            box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
          } else {
            box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
          }
        } 
        else {
          box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;  
        }
      } 
      else if(pcd_box_xmin >= 0 && pcd_box_xmax >= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
        if(box_near_pcd_msg_-> y <= y_r_thd) {
          if((pcd_box_ymax - pcd_box_ymin) < anchor_length) {
            box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
          } else {
            box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
          }
        } else {
          box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
        }
      } 
    }
    //후방
    else if(camera_names_[0] == "am20_rear_center_right") {
      float offset = anchor_length;
      y_r = std::tan(angle_r_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
      y_r_thd = y_r - offset;

      y_l = std::tan(angle_l_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
      y_l_thd = y_l - offset;

      if((pcd_box_xmin <= 0 && pcd_box_xmax >= 0) ||
          (pcd_box_xmin >= -EGO_HALFWIDTH && pcd_box_xmin <= EGO_HALFWIDTH) || 
          (pcd_box_xmax >= -EGO_HALFWIDTH && pcd_box_xmax <= EGO_HALFWIDTH)) {
        box_near_pcd_msg_-> x = pcd_box_xmax - (pcd_box_xmax - pcd_box_xmin)/2;
        box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
      } 
      else if(pcd_box_xmin <= 0 && pcd_box_xmax <= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
        if(box_near_pcd_msg_-> y >= y_r_thd) {
          if((pcd_box_ymax - pcd_box_ymin) < anchor_length) {
            box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
          } else {
            box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
          }
        } 
        else {
          box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;  
        }
      } 
      else if(pcd_box_xmin >= 0 && pcd_box_xmax >= 0) {
        box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
        if(box_near_pcd_msg_-> y >= y_l_thd) {
          if((pcd_box_ymax - pcd_box_ymin) < anchor_length) {
            box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
          } else {
            box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
          }
        } else {
          box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
        }
      } 
    }
    //좌우
    else if(camera_names_[0] == "am20_front_left_rear" ||
        camera_names_[0] == "am20_front_right_rear") {
      float offset = anchor_length;
      // double yup = std::tan(angle_r_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
      // double yup_thd = yup - offset;
      // double ydown = std::tan(-angle_r_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
      // double ydown_thd = ydown + offset;

      if(camera_names_[0] == "am20_front_left_rear") {
        y_r = std::tan(angle_r_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
        y_r_thd = y_r - offset;

        y_l = std::tan(angle_l_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
        y_l_thd = y_l + offset;

        box_near_pcd_msg_-> x = pcd_box_xmax - anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymax - (pcd_box_ymax - pcd_box_ymin)/2;
        if(pcd_box_ymax >= y_r_thd){
          box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
        }
        if(pcd_box_ymax <= y_l_thd){
          box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
        }
      } else {
        y_r = std::tan(angle_r_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
        y_r_thd = y_r + offset;

        y_l = std::tan(angle_l_rad) * std::abs(box_near_pcd_msg_->x+camera2imu_origin_x) + camera2imu_origin_y;
        y_l_thd = y_l - offset;

        box_near_pcd_msg_-> x = pcd_box_xmin + anchor_width/2;
        box_near_pcd_msg_-> y = pcd_box_ymax - (pcd_box_ymax - pcd_box_ymin)/2;
        if(pcd_box_ymax >= y_r_thd){
          box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
        }
        if(pcd_box_ymax <= y_l_thd){
          box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
        }
      }

      // if((pcd_box_ymax - pcd_box_ymin) < anchor_length) {
      //   if(pcd_box_ymax > yup_thd) {
      //     if(pcd_box_ymin < ydown_thd) {
      //       box_near_pcd_msg_-> y = pcd_box_ymax - (pcd_box_ymax - pcd_box_ymin)/2;
      //     } else {
      //       box_near_pcd_msg_-> y = pcd_box_ymin + anchor_length/2;
      //     }
      //   } else {
      //     if(pcd_box_ymin < ydown_thd) {
      //       box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
      //     } else {
      //       box_near_pcd_msg_-> y = pcd_box_ymax - (pcd_box_ymax - pcd_box_ymin)/2;
      //     }
      //   }
      // }
      // else {
      //   box_near_pcd_msg_-> y = pcd_box_ymax - anchor_length/2;
      // }
    }

    if(viz_switch) {
      // float trans_x = 0.5*top_view_width + (box_near_pcd_msg_-> x)*top_view_width/x_range;
      // float trans_y = top_view_height - (box_near_pcd_msg_-> y)*top_view_height/y_range;

      // 박스 중심점
      // cv::circle(top_view_img, cv::Point(trans_x, trans_y), 5, cv::Scalar(0,0,0), -1);

#ifdef GRADIENT      
      if (box_near_pcd_msg_-> label == base::ObjectType::VEHICLE) {
        occlusion_filter = std::sqrt(CAR_LENGTH * CAR_LENGTH + CAR_WIDTH * CAR_WIDTH);
      }
      else if(box_near_pcd_msg_-> label == base::ObjectType::BICYCLE) {
        occlusion_filter = std::sqrt(BICYCLE_LENGTH * BICYCLE_LENGTH + BICYCLE_WIDTH * BICYCLE_WIDTH);
      }
      else if(box_near_pcd_msg_-> label== base::ObjectType::PEDESTRIAN) {
        occlusion_filter = std::sqrt(PEDESTRIAN_LENGTH * PEDESTRIAN_LENGTH + PEDESTRIAN_WIDTH * PEDESTRIAN_WIDTH);
      }
      cv::line(top_view_img, 
          cv::Point(0.5*top_view_width + (box_near_pcd_msg_->x - 1)*top_view_width/x_range, 
              top_view_height - (gradient_distance)*top_view_height/y_range),
          cv::Point(0.5*top_view_width + (box_near_pcd_msg_->x + 1)*top_view_width/x_range, 
              top_view_height - (gradient_distance)*top_view_height/y_range), 
          cv::Scalar(125, 0, 0), 2);
      cv::line(top_view_img, 
          cv::Point(0.5*top_view_width + (box_near_pcd_msg_->x - 1)*top_view_width/x_range, 
              top_view_height - (gradient_distance+occlusion_filter)*top_view_height/y_range),
          cv::Point(0.5*top_view_width + (box_near_pcd_msg_->x + 1)*top_view_width/x_range, 
              top_view_height - (gradient_distance+occlusion_filter)*top_view_height/y_range), 
          cv::Scalar(0, 0, 125), 2);
#endif          
    }

    box_near_pcd_msgs_.push_back(std::move(box_near_pcd_msg_));
    anchor_width_vec.push_back(anchor_width);
    anchor_length_vec.push_back(anchor_length);
    anchor_height_vec.push_back(anchor_height);
  }

  // AERROR << "SIZE : " << in_box_message->perception_obstacle_size() << ", " << box_near_pcd_msgs_.size();
  base::FramePtr bp_cam_fusion_frame(new base::Frame());
  int index = 0;
  for(auto& near_point_ : box_near_pcd_msgs_) {
    base::ObjectPtr obj(new base::Object);

#ifdef USE_LSHAPE
    float lShape_heading;
    bool lShape_state;
    for (auto& box : BoxesInfo) {
      if (box->id == near_point_->id) {
        // TODO :: add the process.

        CalcHeading(box->box_point3, box->box_point4, pose, lShape_heading);
        // lShape_heading = lShape_heading * 180 / M_PI;
        lShape_state = box->lshape_state;
        if (viz_switch) {
          // front
          cv::line(top_view_img, 
                  cv::Point(0.5*top_view_width + (box->box_point1.x)*top_view_width/x_range, 
                      top_view_height - (box->box_point1.y)*top_view_height/y_range),
                  cv::Point(0.5*top_view_width + (box->box_point2.x)*top_view_width/x_range, 
                      top_view_height - (box->box_point2.y)*top_view_height/y_range), 
                  cv::Scalar(0, 255, 0), 2);
          // left
          cv::line(top_view_img,
                  cv::Point(0.5*top_view_width + (box->box_point2.x)*top_view_width/x_range, 
                            top_view_height - (box->box_point2.y)*top_view_height/y_range),
                  cv::Point(0.5*top_view_width + (box->box_point3.x)*top_view_width/x_range, 
                            top_view_height - (box->box_point3.y)*top_view_height/y_range), 
                  cv::Scalar(255, 0, 0), 2);

          // rear
          cv::line(top_view_img, 
                  cv::Point(0.5*top_view_width + (box->box_point3.x)*top_view_width/x_range, 
                      top_view_height - (box->box_point3.y)*top_view_height/y_range),
                  cv::Point(0.5*top_view_width + (box->box_point4.x)*top_view_width/x_range, 
                      top_view_height - (box->box_point4.y)*top_view_height/y_range), 
                  cv::Scalar(255, 0, 0), 2);

          // right
          cv::line(top_view_img, 
                  cv::Point(0.5*top_view_width + (box->box_point4.x)*top_view_width/x_range, 
                      top_view_height - (box->box_point4.y)*top_view_height/y_range),
                  cv::Point(0.5*top_view_width + (box->box_point1.x)*top_view_width/x_range, 
                      top_view_height - (box->box_point1.y)*top_view_height/y_range), 
                  cv::Scalar(255, 0, 0), 2);

          // cv::putText(top_view_img, std::to_string(lShape_heading),
          //             cv::Point(0.5*top_view_width + (box->box_center.x)*top_view_width/x_range+50,
          //                       top_view_height - (box->box_center.y)*top_view_height/y_range),
          //                       1, 2, cv::Scalar(255, 0, 0));
        }
        break; // # To operate just once!
      }
    }
#endif // USE_LSHAPE

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
    // obj->type = near_point_->label;
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

    obj->anchor_point = obj->center;

    Eigen::Vector3f velocity(0.0f, 0.0f, 0.0f);
    obj->velocity = velocity;

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

#ifdef USE_LSHAPE
    lane_heading = static_cast<float>(std::atan2(obj->direction[1], obj->direction[0]));
    float diff_angle;
    CalDiffAngle(lane_heading, lShape_heading, diff_angle);
    if (diff_angle > (M_PI / 4)) {
      lShape_heading += M_PI/2;
    } else if (diff_angle < (-M_PI / 4)) {
      lShape_heading -= M_PI/2;
    }
    if (lShape_state) {
      obj->theta = static_cast<float>(lShape_heading);
    } else {
      obj->theta = static_cast<float>(atan2(obj->direction[1], obj->direction[0]));
    }
#else // USE_LSHAPE
    obj->theta = static_cast<float>(atan2(obj->direction[1], obj->direction[0]));
#endif // USE_LSHAPE

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
      // cv::rectangle(top_view_img, 
      //   cv::Point( 0.5*top_view_width + (view_x0)*top_view_width/x_range, top_view_height - (view_y0)*top_view_height/y_range),
      //   cv::Point( 0.5*top_view_width + (view_x2)*top_view_width/x_range, top_view_height - (view_y2)*top_view_height/y_range),
      //   cv::Scalar(0, 0, 0), 2);

      // std::string text = std::to_string(near_point_->id);
      // cv::putText(top_view_img, text, 
      //   cv::Point(0.5*top_view_width + (view_x0 + view_x2)*0.5*top_view_width/x_range-10, top_view_height - (view_y0 + view_y2)*0.5*top_view_height/y_range-30),
      //   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
    }
  }

  if(viz_switch){
    cv::line(front_view_img, 
      cv::Point(0, 1080),
      cv::Point(1920, 1080), 
      cv::Scalar(0, 0, 0), 10);
    cv::Mat total_img;
    // cv::hconcat(top_view_img, front_view_img, total_img);
    cv::vconcat(front_view_img, top_view_img, total_img);

    std::string img_time = std::to_string(Time::Now().ToNanosecond());
    
    // std::string front_view_name = sub_lidar_fusion_name + "Front View";
    // std::string top_view_name = sub_lidar_fusion_name + "Top View";

    // cv::namedWindow(front_view_name, cv::WINDOW_NORMAL);
    // cv::resizeWindow(front_view_name, 1000, 600);
    // cv::imshow(front_view_name, front_view_img);
    // cv::waitKey(10);

    // cv::namedWindow(top_view_name, cv::WINDOW_NORMAL);
    // cv::resizeWindow(top_view_name, 850, 1000);
    // cv::imshow(top_view_name, top_view_img);
    // cv::waitKey(10);

    cv::namedWindow("Total", cv::WINDOW_NORMAL);
    cv::resizeWindow("Total", 710, 1080);
    cv::imshow("Total", total_img);
    cv::waitKey(10);
    //###
    // cv::imwrite("/apollo/data/output_front_view/" + img_time + ".jpg", front_view_img);
    // cv::imwrite("/apollo/data/output_top_view/" + img_time + ".jpg", top_view_img);
    cv::imwrite("/apollo/data/output_total_view/" + img_time + ".jpg", total_img);
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
    f_x = intrinsic(0,0);
    AINFO << "#intrinsics of " << camera_name << ": "
          << intrinsic_map_[camera_name];
    Eigen::Matrix4d cam_extrinsic;

    std::string cam_name ;
    std::string lid_extrinsic_path ;
    AERROR << "camera_name : " << camera_name;
    if( camera_name == "am20_front_center_right_down"){
      cam_name = "cam_a_1";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_a_extrinsics.yaml";

      angle_l_rad = (90 + camera_angle*0.5) * (CV_PI / 180.0);
      angle_r_rad = (90 - camera_angle*0.5) * (CV_PI / 180.0);
    }
    else if( camera_name == "am20_front_right_rear"){
      cam_name = "cam_a_2";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_b_extrinsics.yaml";

      angle_l_rad = (340 + camera_angle*0.5) * (CV_PI / 180.0);
      angle_r_rad = (340 - camera_angle*0.5) * (CV_PI / 180.0);
    }
    else if( camera_name == "am20_rear_center_right"){
      cam_name = "cam_a_3";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_c_extrinsics.yaml";

      angle_l_rad = (270 + camera_angle*0.5) * (CV_PI / 180.0);
      angle_r_rad = (270 - camera_angle*0.5) * (CV_PI / 180.0);
    }
    else if( camera_name == "am20_front_left_rear"){
      cam_name = "cam_a_4";
      lid_extrinsic_path = "/apollo/modules/drivers/lidar/hesai/params/eth_e_d_extrinsics.yaml";
      
      // angle_l_rad = (200 + camera_angle*0.5) * (CV_PI / 180.0);
      // angle_r_rad = (200 - camera_angle*0.5) * (CV_PI / 180.0);
      angle_l_rad = (200 + camera_angle*0.5) * (CV_PI / 180.0);
      angle_r_rad = (200 - camera_angle*0.5) * (CV_PI / 180.0);
    }

    LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + cam_name +
                       "_extrinsics.yaml",
                   &cam_extrinsic);
    extrinsic_map_[camera_name] = cam_extrinsic;

    Eigen::Matrix<double, 3, 4> cam_extrinsic_34d;
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

  // camera origin → imu
  Eigen::Matrix<double, 3, 1> camera_origin;
  camera_origin << 0, 0, 0;

  Eigen::Matrix<double, 3, 3> imu2cameraMatrix_33d = imu2cameraMatrix_map_[camera_names_[0]].block<3, 3>(0, 0);
  Eigen::Matrix<double, 3, 1> imu2cameraMatrix_31d = imu2cameraMatrix_map_[camera_names_[0]].col(3);

  camera2imu_origin_x = (imu2cameraMatrix_33d.inverse() * (camera_origin - imu2cameraMatrix_31d))(0);
  camera2imu_origin_y = (imu2cameraMatrix_33d.inverse() * (camera_origin - imu2cameraMatrix_31d))(1);


  return true;
  }


  void SwmBpCamFusionComponent::CalcHeading(cv::Point2f& point1, cv::Point2f& point2, Eigen::Affine3d& pose, float& heading) {
    Eigen::Vector3d convert_point1 = pose * Eigen::Vector3d(point1.x, point1.y, 1.0);
    Eigen::Vector3d convert_point2 = pose * Eigen::Vector3d(point2.x, point2.y, 1.0);
    float base_length = convert_point2[0] - convert_point1[0];
    float height_length = convert_point2[1] - convert_point1[1];
    heading = std::atan2(height_length, base_length) + M_PI/2;
    if (heading > M_PI) {
      heading -= 2*M_PI;
    }
  }


  void SwmBpCamFusionComponent::CalDiffAngle(float angle1, float angle2, float& diff_angle) {
    diff_angle = angle1 - angle2;
    if (diff_angle > M_PI) {
      diff_angle -= 2 * M_PI;
    } else if (diff_angle < -M_PI) {
      diff_angle += 2 * M_PI;
    }
  }


}  // namespace onboard
}  // namespace perception
}  // namespace apollo
