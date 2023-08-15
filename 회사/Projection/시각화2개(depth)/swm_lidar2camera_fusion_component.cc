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

    cv::Mat top_view_img(top_view_height, top_view_width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat front_view_img(1080, 1920, CV_8UC3, cv::Scalar(255, 255, 255));

    // std::vector<cv::Scalar> colors = {cv::Scalar(0, 0, 255), 
    // cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(255,255,0), cv::Scalar(255,0,255)};
  // }

  AERROR << "Frame........";
  
  box_roi_pcd_msgs_.clear();
  box_near_pcd_msgs_.clear();

  box_w_map_.clear();

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
        cv::circle(front_view_img, cv::Point(nomal_x, nomal_y), 10, color, -1);
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
        
        cv::circle(top_view_img, 
          cv::Point(0.5*top_view_width + point.x()*top_view_width/x_range, top_view_height - point.y()*top_view_height/y_range), 
          5, color, 2);
      }
      ////
      box_roi_pcd_msgs_.push_back(std::move(box_roi_pcd_msg_));

      break;
      }
      box_id++;
     }
    }


    for (int i =0 ; i < in_box_message->perception_obstacle_size();i++) {

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

    Eigen::Matrix<double, 4, 1> near_point_41d = Eigen::Matrix<double, 4, 1> ::Identity();
    Eigen::Matrix<double, 3, 1> near_proj_point_31d ;

    near_point_41d << box_near_pcd_msg_->x, box_near_pcd_msg_->y, box_near_pcd_msg_->z, 1;
    near_proj_point_31d = resultMatrix_map_[camera_names_[0]] * near_point_41d ;

    Eigen::Matrix<double, 3, 3> resultMatrix_33d = resultMatrix_map_[camera_names_[0]].block<3, 3>(0, 0);
    Eigen::Matrix<double, 3, 1> resultMatrix_31d = resultMatrix_map_[camera_names_[0]].col(3);

    auto depth = near_proj_point_31d(2);

    const auto& box = in_box_message->perception_obstacle(i);
    //test
    // cout << "Object " << i << ", depth: " << depth << endl;
    // cout << "xmin = " << box.bbox2d().xmin()
    //           << ", xmax = " << box.bbox2d().xmax() << endl;
    // cout << "ymin = " << box.bbox2d().ymin() << ", ymax = " << box.bbox2d().ymax() << endl;

    Eigen::Matrix<double, 3, 1> img_box_min_31d(depth * box.bbox2d().xmin(), depth * box.bbox2d().ymin(), depth);
    Eigen::Matrix<double, 3, 1> img_box_max_31d(depth * box.bbox2d().xmax(), depth * box.bbox2d().ymax(), depth);

    double box_min_x = ( resultMatrix_33d.inverse() * (img_box_min_31d - resultMatrix_31d) )(0);
    double box_max_x = ( resultMatrix_33d.inverse() * (img_box_max_31d - resultMatrix_31d) )(0);

    double width = (box_max_x - box_min_x)*0.5;
    // auto box_min_y = ( resultMatrix_33d.inverse() * (img_box_min_31d - resultMatrix_31d) )(1);
    // auto box_max_y = ( resultMatrix_33d.inverse() * (img_box_max_31d - resultMatrix_31d) )(1);

    //test
    // cout << "box" << endl;
    // std::cout << "min: (" << box_min_x << ", " << box_min_y << ")" << std::endl;
    // std::cout << "max: (" << box_max_x << ", " << box_max_y << ")" << std::endl;
    // cout << endl;

    box_w_map_[i] = width;
    ////
    
    if(viz_switch){
      // front view //
      Eigen::Matrix<double, 4, 1> box_min_distance_41d = Eigen::Matrix<double, 4, 1> ::Identity();
      Eigen::Matrix<double, 3, 1> near_proj_point_31d = Eigen::Matrix<double, 3, 1> ::Identity();

      box_min_distance_41d << box_near_pcd_msg_-> x, box_near_pcd_msg_-> y, box_near_pcd_msg_-> z,1.0;
      near_proj_point_31d = resultMatrix_map_[camera_names_[0]] * box_min_distance_41d ;
      auto x_coord = std::round( near_proj_point_31d(0)/std::abs(near_proj_point_31d(2)) );
      auto y_coord = std::round( near_proj_point_31d(1)/std::abs(near_proj_point_31d(2)) );

      // point
      cv::circle(front_view_img, cv::Point(x_coord, y_coord), 5, cv::Scalar(0, 0, 0), 20);

      // rectangle
      cv::rectangle(front_view_img, cv::Point(box.bbox2d().xmin(), box.bbox2d().ymin()),
                  cv::Point(box.bbox2d().xmax(), box.bbox2d().ymax()), cv::Scalar(0, 0, 0), 2);

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

      // center point
      cv::circle(top_view_img, 
        cv::Point(trans_x, trans_y), 
        2, cv::Scalar(0, 0, 0), 2);
      // cv::circle(top_view_img, cv::Point(box_near_pcd_msg_-> trans_x, box_near_pcd_msg_-> trans_y), 1, cv::Scalar(0, 0, 0), 1);

      // text
      std::string top_text_sub_label = ObjectSubTypeToString(box_near_pcd_msg_->sub_label);
      cv::putText(top_view_img, top_text_sub_label, cv::Point(trans_x-65, trans_y-60), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);

      std::string top_text_y_coord = std::to_string(box_near_pcd_msg_->y);
      cv::putText(top_view_img, top_text_y_coord, cv::Point(trans_x-75, trans_y-30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
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

    double width = box_w_map_[near_point_->id];
    
    obj->polygon.resize(4);
    
    obj->polygon[0].x = near_point_->x - width;
    obj->polygon[0].y = near_point_->y;
    obj->polygon[1].x = near_point_->x - width;
    obj->polygon[1].y = near_point_->y + 1;
    obj->polygon[2].x = near_point_->x + width;
    obj->polygon[2].y = near_point_->y + 1;
    obj->polygon[3].x = near_point_->x + width;
    obj->polygon[3].y = near_point_->y;
    obj->distance = near_point_->distance;

    obj->type = near_point_->label;
    obj->sub_type = near_point_->sub_label;
    obj->center = Eigen::Vector3d(near_point_->x,near_point_->y,1.0);

    ////
    if(viz_switch){
      cv::rectangle(top_view_img, 
        cv::Point( 0.5*top_view_width + (obj->polygon[0].x)*top_view_width/x_range, top_view_height - (obj->polygon[0].y)*top_view_height/y_range),
        cv::Point( 0.5*top_view_width + (obj->polygon[2].x)*top_view_width/x_range, top_view_height - (obj->polygon[2].y)*top_view_height/y_range),
        cv::Scalar(0, 0, 0), 2);

      std::string text_width = std::to_string(2*width);

      float trans_x = 0.5*top_view_width + (near_point_-> x)*top_view_width/x_range;
      float trans_y = top_view_height - (near_point_-> y)*top_view_height/y_range;

      cv::putText(top_view_img, text_width, cv::Point(trans_x-75, trans_y+30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(125, 125, 125), 2);
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
    std::string img_time = std::to_string(Time::Now().ToNanosecond());

    // std::string front_file_path= "/apollo/data/output_front_view/"+img_time+".jpg";
    // cv::imwrite(front_file_path, front_view_img);

    cv::namedWindow("Front View", cv::WINDOW_NORMAL);
    cv::resizeWindow("Front View", 1000, 600);
    cv::imshow("Front View", front_view_img);
    cv::waitKey(10);

    // std::string top_file_path= "/apollo/data/output_top_view/"+img_time+".jpg";
    // cv::imwrite(top_file_path, top_view_img);

    cv::namedWindow("Top View", cv::WINDOW_NORMAL);
    cv::resizeWindow("Top View", 850, 1000);
    cv::imshow("Top View", top_view_img);
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
