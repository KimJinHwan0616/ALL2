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

namespace apollo {
namespace perception {
namespace onboard {


uint32_t SwmLidar2cameraFusionComponent::s_seq_num_ = 0;
std::mutex SwmLidar2cameraFusionComponent::s_mutex_;

static bool LoadIntrinsics(const std::string &yaml_file,
                          Eigen::Matrix3d *camera_intrinsic,
                          Eigen::Matrix<double, 1, 5> *camera_distortion) {

  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " does not exist!";
    return false;
  }

  YAML::Node node = YAML::LoadFile(yaml_file);
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double s = 0.0;

  double kk1 = 0.0;
  double kk2 = 0.0;
  double kk3 = 0.0;
  double pk1 = 0.0;
  double pk2 = 0.0;

  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    // qw = node["transform"]["rotation"]["w"].as<double>();
    fx = node["K"][0].as<double>();
    s = node["K"][1].as<double>();
    cx = node["K"][2].as<double>();
    fy = node["K"][4].as<double>();
    cy = node["K"][5].as<double>();

    kk1 = node["D"][0].as<double>();
    kk2 = node["D"][1].as<double>();
    kk3 = node["D"][2].as<double>();
    pk1 = node["D"][3].as<double>();
    pk2 = node["D"][4].as<double>();

  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera intrinsic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera intrinsic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera intrinsic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }

  // camera_intrinsic: 영행렬(3x3)
  camera_intrinsic->setConstant(0);

  // camera_distortion: 영행렬(1x5)
  camera_distortion->setConstant(0);

  (*camera_intrinsic)(0, 0) = fx;
  (*camera_intrinsic)(0, 1) = s;
  (*camera_intrinsic)(0, 2) = cx;
  (*camera_intrinsic)(1, 1) = fy;
  (*camera_intrinsic)(1, 2) = cy;
  (*camera_intrinsic)(2, 2) = 1;

  (*camera_distortion)(0) = kk1;
  (*camera_distortion)(1) = kk2;
  (*camera_distortion)(2) = kk3;
  (*camera_distortion)(3) = pk1;
  (*camera_distortion)(4) = pk2;

  // 확인
  // std::cout << "Intrinsic Matrix"<< std::endl;
  // for (int i = 0; i < 3; ++i) {
  // for (int j = 0; j < 3; ++j) {
  //   std::cout << (*camera_intrinsic)(i, j) << " ";
  // }
  // std::cout << std::endl;
  // }

  // 확인
  // std::cout << "Distortion Matrix"<< std::endl;
  // for (int i = 0; i < 1; ++i) {
  // for (int j = 0; j < 5; ++j) {
  //   std::cout << (*camera_distortion)(i, j) << " ";
  // }
  // std::cout << std::endl;
  // }

    return true;
}

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

  // camera_extrinsic: 영행렬(4x4)
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
  
  // 확인
  // ① imu 외부 파라미터  ② 외부 파라미터

  // std::cout << "rotation" << std::endl;
  // std::cout << "x : " << q.x() << std::endl;
  // std::cout << "y : " << q.y() << std::endl;
  // std::cout << "z : " << q.z() << std::endl;
  // std::cout << "w : " << q.w() << std::endl;

  // std::cout << "translation" << std::endl;
  // std::cout << "x : " << tx << std::endl;
  // std::cout << "y : " << ty << std::endl;
  // std::cout << "z : " << tz << std::endl;

  // std::cout << "Extrinsic Matrix"<< std::endl;
  // for (int i = 0; i < 4; ++i) {
  // for (int j = 0; j < 4; ++j) {
  //   std::cout << (*camera_extrinsic)(i, j) << " ";
  // }
  // std::cout << std::endl;
  // }


  return true;
}

bool SwmLidar2cameraFusionComponent::Init() {
  SwmLidar2cameraFusionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Swm Lidar2camera Fusion Component Configs: " << comp_config.DebugString();

  std::string camera_names_str = comp_config.camera_name();
  boost::algorithm::split(camera_names_, camera_names_str,
                          boost::algorithm::is_any_of(","));

  std::string lidar_names_str = comp_config.lidar_name();
  boost::algorithm::split(lidar_names_, lidar_names_str,
                          boost::algorithm::is_any_of(","));

  box_reader_ = node_->CreateReader<apollo::perception::PerceptionObstacles>(comp_config.input_box_channel_name());

   if(!InitAlgorithmPlugin()) 
   {
    AERROR << "Failed to init algorithm plugin.";
    return false;
   }
  writer_ = node_->CreateWriter<PerceptionObstacles>(
      comp_config.output_obstacles_channel_name());

  return true;
}

bool SwmLidar2cameraFusionComponent::Proc(const std::shared_ptr<PointCloud>& message) {
  box_reader_->Observe();

  // 입력 메세지: box
  auto in_box_message = box_reader_->GetLatestObserved();
  if (in_box_message == nullptr) {
   AERROR << "No detected box";
   in_box_message = box_reader_->GetLatestObserved();
  }

  // 출력 메시지: ??
  auto out_message = std::make_shared<PerceptionObstacles>();
  if(!InternalProc(message, in_box_message, out_message)){
    AERROR << "cam2lidar fusion fail." ;
    return false;
    }
  // writer_->Write(out_message);
  return true;
}

// auto start_time = std::chrono::system_clock::now();

bool SwmLidar2cameraFusionComponent::InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_pcd_message,
                                                  const std::shared_ptr<PerceptionObstacles>& in_box_message,
                                                  const std::shared_ptr<PerceptionObstacles>& out_message){
  box_roi_pcd_msgs_.clear();

  // ①
  // pixel point(3x1): 선언
  Eigen::Matrix<double, 3, 1> projection_matrix_31d;

  //// Camera point(3x1): 선언
  Eigen::Matrix<double, 3, 1> camera_points;

  ////////////////////////////////////////////////////
  // cam_intrinsic Matrix(3x3): 선언
  Eigen::Matrix3d cam_intrinsic;
  Eigen::Matrix<double, 1, 5> camera_distortion;

  // cam_intrinsic Matrix(3x3): 초기화
  LoadIntrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + "cam_a_1" +
                      "_intrinsics.yaml",
                  &cam_intrinsic, &camera_distortion);

  double k1 = camera_distortion(0);
  double k2 = camera_distortion(1);
  double k3 = camera_distortion(2);
  double p1 = camera_distortion(3);
  double p2 = camera_distortion(4);

  // 반복문 출력됨
  // std::cout << k1 << "\n";
  // std::cout << k2 << "\n";
  // std::cout << k3 << "\n";
  // std::cout << p1 << "\n";
  // std::cout << p2 << "\n";
  ////////////////////////////////////////////////////

  ////
  box_width = 1.87;
  offset_top = 5;
  offset_width = box_width/2 + 2.5;
  offset_bottom = 0.1;

  offset_front = 3.5 + 2.5; // 
  ////

  // 
  for (auto point : in_pcd_message->point()) {
    //// 조건문 추가
    if (point.z() < offset_top && 
        offset_bottom < point.z() && 
        point.y() < offset_front &&
        -offset_width < point.x() && point.x() < offset_width
        ) {

      // homo point Matrix(4x1): 선언
      // Eigen::Matrix<double, 4, 1>  bp_projection_41d = Eigen::Matrix<double, 4, 1> ::Identity();
      Eigen::Matrix<double, 4, 1>  bp_projection_41d;

      // homo point Matrix(4x1): 초기화
      // Eigen 초기화: <<(o), =(X)
      bp_projection_41d << point.x(), point.y(), point.z(), 1;

      // resultMatrix_map_[camera_name] = resultMatrix;
      projection_matrix_31d = resultMatrix_map_[camera_names_[0]] * bp_projection_41d ;

      //// 
      // Camera point(3x1): 초기화
      camera_points = extrinsic_distor_map_[camera_names_[0]] * bp_projection_41d;

      // distortion
      auto normal_u = camera_points(0)/std::abs(camera_points(2));
      auto normal_v = camera_points(1)/std::abs(camera_points(2));

      double r_square = normal_u * normal_u + normal_v * normal_v;

      double distor_u = (1 + k1*r_square + k2*r_square*r_square + k3*r_square*r_square*r_square) * normal_u + 2*p1*normal_u*normal_v + p2* (r_square + 2*normal_u*normal_u);
      double distor_v = (1 + k1*r_square + k2*r_square*r_square + k3*r_square*r_square*r_square) * normal_v + p1*(r_square + 2*normal_v*normal_v) + 2*p2*normal_u*normal_v;

      // uv_points(3x1): 선언 + 초기화
      Eigen::Matrix<double, 3, 1> uv_points;
      uv_points << distor_u, distor_v, 1;

      // pixel_points(3x1): 선언 + 초기화
      Eigen::Matrix<double, 3, 1> pixel_points;
      pixel_points = intrinsic_map_[camera_names_[0]].cast<double>() * uv_points;

      // 확인
      // std::cout << "(" << pixel_points(0) << ", " << pixel_points(1) << ", " << pixel_points(2) << ")" << std::endl;
      //

      int box_id = 0;
      for(const auto& box : in_box_message->perception_obstacle()){

        // auto nomal_x = projection_matrix_31d(0)/std::abs(projection_matrix_31d(2));
        // auto nomal_y = projection_matrix_31d(1)/std::abs(projection_matrix_31d(2));

        ////
        auto nomal_x = std::round( pixel_points(0) );
        auto nomal_y = std::round( pixel_points(1) );
        ////

        if( ((box.bbox2d().xmin() <= nomal_x) && ( nomal_x <= box.bbox2d().xmax())) && ((box.bbox2d().ymin() <= nomal_y) && ( nomal_y <= box.bbox2d().ymax())) ){

        // auto box_roi_pcd_msg_ = std::make_shared<PointIL>;
        std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();
        box_roi_pcd_msg_-> x = point.x();
        box_roi_pcd_msg_-> y = point.y();
        box_roi_pcd_msg_-> z = point.z();
        box_roi_pcd_msg_-> id = box_id;
        box_roi_pcd_msg_-> label = box.type();
        box_roi_pcd_msg_-> sub_label = box.sub_type();

        // 확인
        // std::cout << "(" << box_roi_pcd_msg_->x
        //     << "," << box_roi_pcd_msg_->y
        //     << "," << box_roi_pcd_msg_->z
        //     << "," << box_roi_pcd_msg_->id
        //     << "," << box_roi_pcd_msg_->label
        //     << ")" << std::endl;

        // {x, y, z, id, label, sub_label}
        box_roi_pcd_msgs_.push_back(std::move(box_roi_pcd_msg_));

        break;
        }

        box_id++;
      }
    }; 
    }

  return true;
}

// auto end_time = std::chrono::system_clock::now();
// std::chrono::duration<double> diff = end_time - start_time;
// double used_time = diff.count();

// std::cout << used_time;

bool SwmLidar2cameraFusionComponent::InitAlgorithmPlugin() {
  AERROR << "InitAlgorithmPlugin start is ok";
  
  // camera_names = { camera_name, ..., camera_name }
  for (const auto &camera_name : camera_names_) {
    AERROR << camera_name ;

    base::BaseCameraModelPtr model;
    model =
        common::SensorManager::Instance()->GetUndistortCameraModel(camera_name);
        // common::SensorManager::Instance()->GetDistortCameraModel(camera_name);

    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());

    // K(3x3): 선언 + 초기화
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();

    // Key: camera_name, Value: intrinsic
    intrinsic_map_[camera_name] = intrinsic;

    AINFO << "#intrinsics of " << camera_name << ": "
          << intrinsic_map_[camera_name];

    // Camera intrinsic Matrix(3x3): 선언
    Eigen::Matrix3d cam_intrinsic;
    // Camera distortion Matrix(1x5): 선언
    Eigen::Matrix<double, 1, 5> camera_distortion;

    // Camera intrinsic Matrix(3x3): 초기화
    // Camera distortion Matrix(1x5): 초기화
    LoadIntrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + "cam_a_1" +
                       "_intrinsics.yaml",
                   &cam_intrinsic, &camera_distortion);
    //
    // test_map_[camera_name] = cam_intrinsic;
    //

    // Camera extrinsic Matrix(4x4): 선언
    Eigen::Matrix4d cam_extrinsic;

    // Camera extrinsic Matrix: 초기화
    LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + "cam_a_1" +
                       "_extrinsics.yaml",
                   &cam_extrinsic);

    // Key: camera_name, Value: cam_extrinsic
    extrinsic_map_[camera_name] = cam_extrinsic;

    // Camera extrinsic Matrix(3x4)⁻¹: 선언 + 초기화
    Eigen::Matrix<double, 3, 4>  cam_extrinsic_34d;
    cam_extrinsic_34d = cam_extrinsic.inverse().block<3, 4>(0, 0);

    // 확인
    // std::cout << "Camera extrinsic Matrix(3x4)⁻¹"<< std::endl;
    // for (int i = 0; i < 3; ++i) {
    // for (int j = 0; j < 4; ++j) {
    //   std::cout << (cam_extrinsic_34d)(i, j) << " ";
    // }
    // std::cout << std::endl;
    // }

    // imu extrinsic Matrix(4x4): 선언
    Eigen::Matrix4d lid_extrinsic;

    // imu extrinsic Matrix(3x4): 초기화
    LoadExtrinsics("/apollo/modules/drivers/lidar/hesai/params/eth_e_a_extrinsics.yaml",
                   &lid_extrinsic);

    // imu extrinsic Matrix(3x4)⁻¹: 선언 + 초기화
    Eigen::Matrix<double, 4, 4>  lid_extrinsic_44d;
    lid_extrinsic_44d = lid_extrinsic.inverse().block<4, 4>(0, 0);

    // 확인
    // std::cout << "imu extrinsic Matrix(3x4)⁻¹"<< std::endl;
    // for (int i = 0; i < 4; ++i) {
    // for (int j = 0; j < 4; ++j) {
    //   std::cout << (lid_extrinsic_44d)(i, j) << " ";
    // }
    // std::cout << std::endl;
    // }

    //// imu → Camera Matrix(3,4): 선언 + 초기화
    Eigen::Matrix<double, 3, 4> lidar2cameraMatrix;
    lidar2cameraMatrix = cam_extrinsic_34d * lid_extrinsic_44d;

    // 확인
    // std::cout << "lidar2cameraMatrix"<< std::endl;
    // for (int i = 0; i < 3; ++i) {
    // for (int j = 0; j < 4; ++j) {
    //   std::cout << (lidar2cameraMatrix)(i, j) << " ";
    // }
    // std::cout << std::endl;
    // }

    //// Key: camera_name, Value: lidar2cameraMatrix
    extrinsic_distor_map_[camera_name] = lidar2cameraMatrix;

    // Projection Matrix(3x4): 선언
    Eigen::Matrix<double, 3, 4> resultMatrix;

    // Projection Matrix(3x4): 초기화
    resultMatrix = intrinsic.cast<double>() * cam_extrinsic_34d * lid_extrinsic_44d;

    // Key: camera_name, Value: resultMatrix
    resultMatrix_map_[camera_name] = resultMatrix;
  }
  
  AERROR << "InitAlgorithmPlugin out is ok";
  
  return true;
  }

}  // namespace onboard
}  // namespace perception
}  // namespace apollo