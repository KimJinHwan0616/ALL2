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

#include "opencv2/opencv.hpp"
#include <numeric>

using namespace std;

namespace apollo {
namespace perception {
namespace onboard {

uint32_t SwmLidar2cameraFusionComponent::s_seq_num_ = 0;
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

bool SwmLidar2cameraFusionComponent::InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_pcd_message,
                                                  const std::shared_ptr<PerceptionObstacles>& in_box_message,
                                                  const std::shared_ptr<PerceptionObstacles>& out_message){
  box_roi_pcd_msgs_.clear();
  box_near_pcd_msgs_.clear();
  // pixel point(3x1): 선언
  Eigen::Matrix<double, 3, 1> projection_matrix_31d;

  // box_width = 1.87;
  // offset_top = 5;
  // offset_bottom = 0.1;
  offset_front = 3.5 + 2.5; // 

  std::string time = std::to_string(Time::Now().ToNanosecond());

  //// visualize
  // cv::Mat img(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat img = cv::imread("/apollo/data/input_img/1.jpg");

  if (img.empty()) {
    std::cout << "img load fail";
  }

  AERROR << "Frame........";

  // 프레임에 있는 point 1개
  for (auto point : in_pcd_message->point()) {
    // homo point Matrix(4x1): 선언
    // Eigen::Matrix<double, 4, 1>  bp_projection_41d = Eigen::Matrix<double, 4, 1> ::Identity();
    Eigen::Matrix<double, 4, 1>  bp_projection_41d;

    // homo point Matrix(4x1): 초기화
    // Eigen 초기화: <<(o), =(X)
    bp_projection_41d << point.x(), point.y(), point.z(), 1;

    // resultMatrix_map_[camera_name] = resultMatrix;
    projection_matrix_31d = resultMatrix_map_[camera_names_[0]] * bp_projection_41d ;

    unsigned int box_id = 0;

    //// visualize
    std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 0, 255), 
      cv::Scalar(0, 255, 0),
      cv::Scalar(255, 0, 0),
      cv::Scalar(255,255,0),
      cv::Scalar(255,0,255)
      };
    ////
    
    // 박스안에 있는 point 1개
    for(auto& box : in_box_message->perception_obstacle()){

      auto nomal_x = std::round( projection_matrix_31d(0)/std::abs(projection_matrix_31d(2)) );
      auto nomal_y = std::round( projection_matrix_31d(1)/std::abs(projection_matrix_31d(2)) );

      if( ((box.bbox2d().xmin() <= nomal_x) && ( nomal_x <= box.bbox2d().xmax())) && ((box.bbox2d().ymin() <= nomal_y) && ( nomal_y <= box.bbox2d().ymax())) ){

        //// visualize
        cv::Scalar color;
        if (box_id < colors.size()) {
            color = colors[box_id];
        } else {
            color = cv::Scalar(255, 255, 255);  // White color for extra boxes
        }

        cv::circle(img, cv::Point(nomal_x, nomal_y), 5, color, -1);
        ////

        // auto box_roi_pcd_msg_ = std::make_shared<PointIL>;
        std::shared_ptr<PointIL> box_roi_pcd_msg_ = std::make_shared<PointIL>();

        box_roi_pcd_msg_-> x = point.x();
        box_roi_pcd_msg_-> y = point.y();
        box_roi_pcd_msg_-> z = point.z();
        box_roi_pcd_msg_-> id = box_id;
        box_roi_pcd_msg_-> distance = std::sqrt(point.x()*point.x() + point.y()*point.y() + point.z()*point.z());
        box_roi_pcd_msg_-> label = static_cast<base::ObjectType>(box.type());
        box_roi_pcd_msg_-> sub_label = static_cast<base::ObjectSubType>(box.sub_type());

        // {x, y, z, id, label, sub_label}
        box_roi_pcd_msgs_.push_back(std::move(box_roi_pcd_msg_));
        break;
      }
      box_id++;
    }
  }

  std::vector<std::shared_ptr<PointIL>> box_msgs;

  // ①
  // for (int i =0 ; i < in_box_message->perception_obstacle_size();i++){
  //   float near_point = 100.0;
  //   float far_point = 0.0;
  //   std::shared_ptr<PointIL> box_near_pcd_msg_ = std::make_shared<PointIL>();
  //   std::shared_ptr<PointIL> box_far_pcd_msg_ = std::make_shared<PointIL>();
  //   std::shared_ptr<PointIL> box_middle_pcd_msg_ = std::make_shared<PointIL>(); 

  //   for (const auto& box_ : box_roi_pcd_msgs_ ){
  //     if (box_->id == i){
  //       if(box_->distance < near_point){
  //         near_point = box_->distance ;
  //         box_near_pcd_msg_-> x = box_->x;
  //         box_near_pcd_msg_-> y = box_->y;
  //         box_near_pcd_msg_-> z = box_->z;
  //         box_near_pcd_msg_-> distance = box_->distance;
  //         box_near_pcd_msg_-> id = box_->id;
  //         box_near_pcd_msg_-> label = box_->label;
  //         box_near_pcd_msg_-> sub_label = box_->sub_label;
  //       }

  //       if (box_->distance > far_point) {
  //       far_point = box_->distance;
  //       box_far_pcd_msg_->x = box_->x;
  //       box_far_pcd_msg_->y = box_->y;
  //       box_far_pcd_msg_->z = box_->z;
  //       box_far_pcd_msg_->distance = box_->distance;
  //       box_far_pcd_msg_->id = box_->id;
  //       box_far_pcd_msg_->label = box_->label;
  //       box_far_pcd_msg_->sub_label = box_->sub_label;
  //       }
  //     }
  //   }

  //   box_middle_pcd_msg_->x = (box_near_pcd_msg_->x + box_far_pcd_msg_->x) / 2;
  //   box_middle_pcd_msg_->y = (box_near_pcd_msg_->y + box_far_pcd_msg_->y) / 2;
  //   box_middle_pcd_msg_->z = (box_near_pcd_msg_->z + box_far_pcd_msg_->z) / 2;
  //   box_middle_pcd_msg_->distance = (box_near_pcd_msg_->distance + box_far_pcd_msg_->distance) / 2;
  //   box_middle_pcd_msg_->id = i;

  //   // if (box_near_pcd_msg_->id == 0) {
  //   std::cout << "Object " << i << std::endl;
  //   std::cout << "Minimum Distance: " << box_near_pcd_msg_->distance << std::endl;
  //   // std::cout << "y: " << box_near_pcd_msg_->y << std::endl;
  //   std::cout << "Minimum Coordinates: (" << box_near_pcd_msg_->x << ", " << box_near_pcd_msg_->y << ", " << box_near_pcd_msg_->z << ")" << std::endl;

  //   std::cout << "Maximum Distance: " << box_far_pcd_msg_->distance << std::endl;
  //   std::cout << "Maximum Coordinates: (" << box_far_pcd_msg_->x << ", " << box_far_pcd_msg_->y << ", " << box_far_pcd_msg_->z << ")" << std::endl;
  //   // }
  //   // box_msgs.push_back(box_near_pcd_msg_);
  //   // box_msgs.push_back(box_far_pcd_msg_);
  //   box_msgs.push_back(box_middle_pcd_msg_); 
  // }

  // ②
  int num_nearest_points = 15;

  for (int i = 0; i < in_box_message->perception_obstacle_size(); i++) {

    std::vector<float> near_points(num_nearest_points, 100.0);
    std::vector<std::shared_ptr<PointIL>> box_near_pcd_msgs(num_nearest_points);

    for (const auto& box_ : box_roi_pcd_msgs_) {
        if (box_->id == i) {

            for (int j = 0; j < num_nearest_points; j++) {
                if (box_->distance < near_points[j]) {
                    for (int k = num_nearest_points - 1; k > j; k--) {
                        near_points[k] = near_points[k - 1];
                        box_near_pcd_msgs[k] = box_near_pcd_msgs[k - 1];
                    }
                    near_points[j] = box_->distance;
                    box_near_pcd_msgs[j] = std::make_shared<PointIL>(*box_);
                    break;
                }
            }
        }
    }

    float average_x = 0.0;
    float average_y = 0.0;
    float average_z = 0.0;

    for (int j = 0; j < num_nearest_points; j++) {
        // std::cout << "Object " << i << ", Closest Point " << j + 1 << std::endl;
        // std::cout << "Minimum Distance: " << box_near_pcd_msgs[j]->distance << std::endl;
        // std::cout << "Minimum Coordinates: (" << box_near_pcd_msgs[j]->x << ", "
        //           << box_near_pcd_msgs[j]->y << ", " << box_near_pcd_msgs[j]->z << ")" << std::endl;

        average_x += box_near_pcd_msgs[j]->x;
        average_y += box_near_pcd_msgs[j]->y;
        average_z += box_near_pcd_msgs[j]->z;
    }

    average_x /= num_nearest_points;
    average_y /= num_nearest_points;
    average_z /= num_nearest_points;

    // std::cout << std::endl;
    std::cout << "Object " << i << endl;
    std::cout << "Average Coordinates: (" << average_x << ", " << average_y << ", " << average_z << ")"
              << std::endl;
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;

    box_msgs.push_back(std::make_shared<PointIL>(PointIL{average_x, average_y, average_z}));
  }

  /////////////////////////////////////////////////
  Eigen::Matrix<double, 4, 1> box_min_distance_41d;
  Eigen::Matrix<double, 3, 1> pixel_coordinate_31d;

  // // Print the minimum distance for each box ID
  for (const auto& box_msg : box_msgs) {
      // if (box_msg->id == 0) {
        box_min_distance_41d(0) = box_msg->x;
        box_min_distance_41d(1) = box_msg->y;
        box_min_distance_41d(2) = box_msg->z;
        box_min_distance_41d(3) = 1.0;

        pixel_coordinate_31d = resultMatrix_map_[camera_names_[0]] * box_min_distance_41d ;

        auto x_coord = std::round( pixel_coordinate_31d(0)/std::abs(pixel_coordinate_31d(2)) );
        auto y_coord = std::round( pixel_coordinate_31d(1)/std::abs(pixel_coordinate_31d(2)) );

        cv::circle(img, cv::Point(x_coord, y_coord), 5, cv::Scalar(0, 255, 255), 20);
      // }
  }

  //// visualize
  // std::string  file_time_path= "/apollo/data/output_img/"+img_time+".jpg";
  // cv::imwrite(file_time_path, img);

  cv::namedWindow("Image", cv::WINDOW_NORMAL);
  cv::resizeWindow("Image", 1440, 810);
  cv::imshow("Image", img);
  cv::waitKey(10);
  ////
  return true;
}

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

    // imu extrinsic Matrix(4x4): 선언
    Eigen::Matrix4d lid_extrinsic;

    // imu extrinsic Matrix(3x4): 초기화
    LoadExtrinsics("/apollo/modules/drivers/lidar/hesai/params/eth_e_a_extrinsics.yaml",
                   &lid_extrinsic);

    // imu extrinsic Matrix(3x4)⁻¹: 선언 + 초기화
    Eigen::Matrix<double, 4, 4>  lid_extrinsic_44d;
    lid_extrinsic_44d = lid_extrinsic.inverse().block<4, 4>(0, 0);

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