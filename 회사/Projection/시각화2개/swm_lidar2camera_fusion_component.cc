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
#include "modules/perception/onboard/component/swm_lidar2camera_fusion_component.h"

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

namespace apollo {
namespace perception {
namespace onboard {

// std::atomic<uint32_t> SwmLidar2cameraFusionComponent::seq_num_{0};
uint32_t SwmLidar2cameraFusionComponent::seq_num_ = 0;
std::mutex SwmLidar2cameraFusionComponent::s_mutex_;

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

bool SwmLidar2cameraFusionComponent::Init() {
  SwmLidar2cameraFusionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Swm Lidar2camera Fusion Component Configs: " << comp_config.DebugString();

  viz_switch = comp_config.viz_switch();
  AERROR << "viz_switch : " << viz_switch;

  std::string camera_names_str = comp_config.camera_name();
  boost::algorithm::split(camera_names_, camera_names_str,
                          boost::algorithm::is_any_of(","));

  std::string lidar_names_str = comp_config.lidar_name();
  boost::algorithm::split(lidar_names_, lidar_names_str,
                          boost::algorithm::is_any_of(","));


  if (!common::SensorManager::Instance()->GetSensorInfo(
          comp_config.sub_lidar_fusion_name(), &lidar_info_)) {
    AERROR << "Failed to get sensor info, sensor name: "
           << comp_config.sub_lidar_fusion_name();
    return false;
  }

  // if (!common::SensorManager::Instance()->GetSensorInfo(
  //         comp_config.camera_name(), &camera_info_)) {
  //   AERROR << "Failed to get sensor info, sensor name: "
  //          << comp_config.camera_name();
  //   return false;
  // }

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
  // writer_ = node_->CreateWriter<SensorFrameMessage>(
  //     comp_config.output_obstacles_channel_name());
  writer_ = node_->CreateWriter<SensorFrameMessage>("/perception/inner/PrefusedObjects");

  box_bp_writer_ = node_->CreateWriter<PointCloud>("perception/test/box_in_bp_data");

  return true;
}

bool SwmLidar2cameraFusionComponent::Proc(const std::shared_ptr<drivers::PointCloud>& message) {
  // AERROR << "Proc";
  // box_reader_->Observe();
  // auto in_box_message = box_reader_->GetLatestObserved();
  // if (in_box_message == nullptr) {
  //  AERROR << "No detected box";
  //  in_box_message = box_reader_->GetLatestObserved();
  // }
  #ifndef shm_bp 
  // rsbp_reader_->Observe();
  // auto in_rsbp_message = rsbp_reader_->GetLatestObserved();
  
  // if (in_rsbp_message == nullptr) {
  //  std::this_thread::sleep_for(std::chrono::seconds(1));
  //  in_rsbp_message = rsbp_reader_->GetLatestObserved();
  // }
  // // if (in_rsbp_message == nullptr) {
  // //  std::this_thread::sleep_for(std::chrono::seconds(1));
  // // //  AERROR << "No detected rsbp pointcloud";
  // //  in_rsbp_message = rsbp_reader_->GetLatestObserved();
  // // }if (in_rsbp_message == nullptr) {
  // //  std::this_thread::sleep_for(std::chrono::seconds(1));
  // // //  AERROR << "No detected rsbp pointcloud";
  // //  in_rsbp_message = rsbp_reader_->GetLatestObserved();
  // // }
  // if (in_rsbp_message == nullptr) {
  // return true;
  // }
  box_reader_->Observe();
  auto in_box_message = box_reader_->GetLatestObserved();
  
  if (in_box_message == nullptr) {
   std::this_thread::sleep_for(std::chrono::seconds(1));
   in_box_message = box_reader_->GetLatestObserved();
  }
  // if (in_box_message == nullptr) {
  //  std::this_thread::sleep_for(std::chrono::seconds(1));
  // //  AERROR << "No detected rsbp pointcloud";
  //  in_box_message = box_reader_->GetLatestObserved();
  // }if (in_box_message == nullptr) {
  //  std::this_thread::sleep_for(std::chrono::seconds(1));
  // //  AERROR << "No detected rsbp pointcloud";
  //  in_box_message = box_reader_->GetLatestObserved();
  // }
  if (in_box_message == nullptr) {
  return true;
  }


  // while(in_rsbp_message == nullptr) {
  // //  std::this_thread::sleep_for(std::chrono::seconds(1));
  //  AERROR << "No detected rsbp pointcloud";
  //  in_rsbp_message = rsbp_reader_->GetLatestObserved();
  // }
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

      box_bp_writer_->Write(in_rsbp_message);

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

bool SwmLidar2cameraFusionComponent::InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_pcd_message,
                                                  const std::shared_ptr<PerceptionObstacles>& in_box_message,
                                                  const std::shared_ptr<SensorFrameMessage>& out_message){                                         
  //tf
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_novatel = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      // in_pcd_message->measurement_time();// - lidar_query_tf_offset_ * 0.001;
      in_pcd_message->header().timestamp_sec();//hoseob

  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose,
                                               &pose_novatel)) {

    while(lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose,
                                               &pose_novatel)){
      lidar_query_tf_timestamp + 0.01;
                                               }

    return true;
  } 
  //tf
  // if(viz_switch){
    uint16_t top_view_width = 1000;
    uint16_t top_view_height = 2000;
    uint8_t x_range = 50;
    uint8_t y_range = 100;

    cv::Mat top_view_image(top_view_height, top_view_width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat front_view_img(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));

    // std::vector<cv::Scalar> colors = {cv::Scalar(0, 0, 255), 
    // cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(255,255,0), cv::Scalar(255,0,255)};
  // }

  AERROR << "Frame........";
  
  box_roi_pcd_msgs_.clear();
  box_near_pcd_msgs_.clear();
  box_pcd_data = std::make_shared<PointCloud>();
  Eigen::Matrix<double, 3, 1>  projection_matrix_31d ;
  for (auto point : in_pcd_message->point()) {
    // if (point.y() >=7 || 6 <= point.z() || 0.0 >= point.z()) continue;
    if ( 0.0 >= point.z()) continue;

    Eigen::Matrix<double, 4, 1>  bp_projection_41d = Eigen::Matrix<double, 4, 1> ::Identity();
    bp_projection_41d << point.x(), point.y(), point.z(), 1;
    projection_matrix_31d = resultMatrix_map_[camera_names_[0]] * bp_projection_41d ;

    int box_id = 0;
    for(auto& box : in_box_message->perception_obstacle()){

      auto nomal_x = projection_matrix_31d(0)/std::abs(projection_matrix_31d(2));
      auto nomal_y = projection_matrix_31d(1)/std::abs(projection_matrix_31d(2));

      if(((box.bbox2d().xmin() <= nomal_x) && ( nomal_x <= box.bbox2d().xmax())) && ((box.bbox2d().ymin() <= nomal_y) && ( nomal_y <= box.bbox2d().ymax()))){
      
      // front view //
      if(viz_switch){
        cv::Scalar color;
        if (box_id < (int)colors.size()) {
            color = colors[box_id];
        } else {
            color = cv::Scalar(125, 125, 125);  
        }
        //// box 수정 중
        cv::rectangle(front_view_img, cv::Point(box.bbox2d().xmin(), box.bbox2d().ymin()),
                      cv::Point(box.bbox2d().xmax(), box.bbox2d().ymax()), color, 2);
        ////
        cv::circle(front_view_img, cv::Point(nomal_x, nomal_y), 2, color, -1);
      }

      std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
      box_roi_pcd_msg_-> x = point.x();
      box_roi_pcd_msg_-> y = point.y();
      box_roi_pcd_msg_-> z = point.z();
      box_roi_pcd_msg_-> distance = std::sqrt(point.x()*point.x() + point.y()*point.y());
      box_roi_pcd_msg_-> id = box_id;
      box_roi_pcd_msg_-> label = static_cast<base::ObjectType>(box.type());
      box_roi_pcd_msg_-> sub_label = static_cast<base::ObjectSubType>(box.sub_type());
      ////
      // top view //
      if(viz_switch){
        cv::Scalar color;
        if (box_id < (int)colors.size()) {
            color = colors[box_id];
        } else {
            color = cv::Scalar(125, 125, 125);  // White color for extra boxes
        }
        cv::circle(top_view_image, 
          cv::Point(0.5*top_view_width + point.x()*top_view_width/x_range, top_view_height - point.y()*top_view_height/y_range), 
          10, color, 10);
      }
      ////
      box_roi_pcd_msgs_.push_back(std::move(box_roi_pcd_msg_));

      break;
      }
      box_id++;
     }
    }

  for  (int i =0 ; i < in_box_message->perception_obstacle_size();i++){
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
          ////

        }
      }
    }
  
    if (near_point == 100.0){continue;}

    if(viz_switch){
      // front view //
      Eigen::Matrix<double, 4, 1> box_min_distance_41d = Eigen::Matrix<double, 4, 1> ::Identity();
      Eigen::Matrix<double, 3, 1> pixel_coordinate_31d = Eigen::Matrix<double, 3, 1> ::Identity();

      box_min_distance_41d << box_near_pcd_msg_-> x, box_near_pcd_msg_-> y, box_near_pcd_msg_-> z,1.0;
      pixel_coordinate_31d = resultMatrix_map_[camera_names_[0]] * box_min_distance_41d ;
      auto x_coord = std::round( pixel_coordinate_31d(0)/std::abs(pixel_coordinate_31d(2)) );
      auto y_coord = std::round( pixel_coordinate_31d(1)/std::abs(pixel_coordinate_31d(2)) );

      // point
      cv::circle(front_view_img, cv::Point(x_coord, y_coord), 5, cv::Scalar(0, 0, 0), 20);

      // text
      std::string front_text_sub_label = ObjectSubTypeToString(box_near_pcd_msg_->sub_label);
      std::string front_text_y_coord = std::to_string(box_near_pcd_msg_->y);

      cv::putText(front_view_img, front_text_sub_label, cv::Point(x_coord-65, y_coord-60), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
      cv::putText(front_view_img, front_text_y_coord, cv::Point(x_coord-75, y_coord-30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
      
      ////////////////////////////////////////////////////////////////////////////////////////////
      // top view //

      // cv::Scalar color;
      // if (box_near_pcd_msg_->id < (int)colors.size()) {
      //     color = colors[box_near_pcd_msg_->id];
      // } else {
      //     color = cv::Scalar(125, 125, 125);  // White color for extra boxes
      // }
      float trans_x = 0.5*top_view_width + (box_near_pcd_msg_-> x)*top_view_width/x_range;
      float trans_y = top_view_height - (box_near_pcd_msg_-> y)*top_view_height/y_range;

      // point
      cv::circle(top_view_image, 
        cv::Point(trans_x, trans_y), 
        10, cv::Scalar(0, 0, 0), 10);
      // cv::circle(top_view_image, cv::Point(box_near_pcd_msg_-> trans_x, box_near_pcd_msg_-> trans_y), 1, cv::Scalar(0, 0, 0), 1);

      // text
      std::string top_text_sub_label = ObjectSubTypeToString(box_near_pcd_msg_->sub_label);
      std::string top_text_y_coord = std::to_string(box_near_pcd_msg_->y);

      cv::putText(top_view_image, top_text_sub_label, cv::Point(trans_x-65, trans_y-60), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
      cv::putText(top_view_image, top_text_y_coord, cv::Point(trans_x-75, trans_y-30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
      ////
    }

    PointXYZIT* point_new = box_pcd_data->add_point();
        point_new->set_x(box_near_pcd_msg_->x);
        point_new->set_y(box_near_pcd_msg_->y);
        point_new->set_z(box_near_pcd_msg_->z);
        point_new->set_intensity(80);
        point_new->set_timestamp(0);

    box_near_pcd_msgs_.push_back(std::move(box_near_pcd_msg_));
  }

  box_bp_writer_->Write(box_pcd_data);

  base::FramePtr lidar2camera_frame(new base::Frame());
  for(auto& near_point_ : box_near_pcd_msgs_){
    base::ObjectPtr obj(new base::Object);
    obj->polygon.resize(4);
    obj->polygon[0].x = near_point_->x - 1 ;
    obj->polygon[0].y = near_point_->y ;
    obj->polygon[1].x = near_point_->x - 1 ;
    obj->polygon[1].y = near_point_->y + 1 ;
    obj->polygon[2].x = near_point_->x + 1 ;
    obj->polygon[2].y = near_point_->y + 1 ;
    obj->polygon[3].x = near_point_->x + 1 ;
    obj->polygon[3].y = near_point_->y ;
    obj->distance = near_point_->distance;

    obj->type = near_point_->label;
    obj->sub_type = near_point_->sub_label;
    obj->center = Eigen::Vector3d(near_point_->x,near_point_->y,1.0);
    ////
    if(viz_switch){
      cv::rectangle(top_view_image, 
      cv::Point( 0.5*top_view_width + (obj->polygon[0].x)*top_view_width/x_range, top_view_height - (obj->polygon[0].y)*top_view_height/y_range),
      cv::Point( 0.5*top_view_width + (obj->polygon[2].x)*top_view_width/x_range, top_view_height - (obj->polygon[2].y)*top_view_height/y_range),
      cv::Scalar(0, 0, 0), 2);
    }
    ////

    /////////////////////////////////////////////////////////////////
    obj->id = nearest_obstacle_id;
    obj->track_id = nearest_obstacle_id;
    nearest_obstacle_id++;
    if(nearest_obstacle_id > 9999) {
      nearest_obstacle_id = 1;
    }
    obj->anchor_point = obj->center;

    Eigen::Vector3f velocity(0.0f, 0.0f, 0.0f);
    obj->velocity = velocity;

    obj->center_uncertainty = Eigen::Matrix3f::Zero();
    obj->velocity_uncertainty = Eigen::Matrix3f::Zero();
    Eigen::Vector3f direction(0.0f, 0.0f, 0.0f);
    obj->direction = direction;
    obj->theta = 0.0f;
    obj->theta_variance = 0.0f;
    obj->confidence = 1.0f;

    obj->motion_state = base::MotionState::UNKNOWN;

    obj->size(0) = 0.05f;
    obj->size(1) = 0.05f;
    obj->size(2) = 2.0f;  // vehicle template (pnc required)
    /////////////////////////////////////////////////////////////////
    lidar2camera_frame->objects.push_back(obj);

    // out_message->frame_->objects.push_back(obj);
  }

  if(viz_switch){
    // std::string img_time = std::to_string(Time::Now().ToNanosecond());
    // std::string file_time_path= "/apollo/data/output_img/"+img_time+".jpg";
    // cv::imwrite(file_time_path, top_view_image);

    // cv::namedWindow("Front View", cv::WINDOW_NORMAL);
    // cv::resizeWindow("Front View", 1000, 600);
    // cv::imshow("Front View", front_view_img);
    // cv::waitKey(10);

    cv::namedWindow("Top View", cv::WINDOW_NORMAL);
    cv::resizeWindow("Top View", 1700, 2000);
    cv::imshow("Top View", top_view_image);
    cv::waitKey(10);
  }

  lidar2camera_frame->timestamp = in_pcd_message->header().timestamp_sec();
  // lidar2camera_frame->sensor2world_pose = *(options.radar2world_pose);

  std::vector<base::ObjectPtr> lidar2camera_objects;
  if (!lidar2camera_perception_->Perceivelidar2camera(lidar2camera_frame,
                                   &lidar2camera_objects)) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "RadarDetector Proc failed.";
    return true;
  }

  ++seq_num_;

  // out_message->timestamp_ = in_pcd_message->measurement_time();

  // AERROR << lidar2camera_objects.size();

  out_message->timestamp_ = in_pcd_message->header().timestamp_sec();//hoseob
  out_message->seq_num_ = seq_num_;
  out_message->process_stage_ = ProcessStage::LONG_RANGE_RADAR_DETECTION;
  out_message->sensor_id_ = "velodyne128";

  out_message->frame_.reset(new base::Frame());

  out_message->frame_->sensor2world_pose = pose;

  out_message->frame_->sensor_info = lidar_info_;
  out_message->frame_->objects = lidar2camera_objects;
  return true;
}

bool SwmLidar2cameraFusionComponent::InitAlgorithmPlugin() {
  // AERROR << "InitAlgorithmPlugin start is ok";

  // std::string pipeline_name_ = "lidar2cameraObstaclePerception";
  // std::string perception_method_ = "Frontlidar2cameraPipeline";
  std::string perception_method_ = "lidar2cameraObstaclePerception";
  std::string pipeline_name_ = "Frontlidar2cameraPipeline";

  lidar2camera::Baselidar2cameraObstaclePerception* lidar2camera_perception =
      lidar2camera::Baselidar2cameraObstaclePerceptionRegisterer::GetInstanceByName(
          perception_method_);
  ACHECK(lidar2camera_perception != nullptr)
      << "No lidar2camera obstacle perception named: " << perception_method_;
  lidar2camera_perception_.reset(lidar2camera_perception);

  ACHECK(lidar2camera_perception_->Init(pipeline_name_))
      << "Failed to init lidar2camera perception.";
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