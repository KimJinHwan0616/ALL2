#include "CVC_cluster.h"

bool compare_cluster(std::pair<int,int> a,std::pair<int,int> b) {
  return a.second>b.second;
}//upper sort

// 직교 좌표계 → 극 좌표계
float Polar_angle_cal(float x, float y) {
  float temp_tangle = 0;  // 계산된 각도를 임시로 저장할 변수

  if(x == 0 && y == 0) {
      temp_tangle = 0;  // 원점의 경우 각도를 0으로 설정
  } else if(y >= 0) {
      temp_tangle = (float)atan2(y, x);  // 양의 y축 위의 경우 각도 계산
  } else if(y < 0) {
      temp_tangle = (float)atan2(y, x) + 2 * M_PI;  // 음의 y축 위의 경우 각도 계산 후 360도 추가
  }

  return temp_tangle;  // 계산된 극 좌표계의 각도 반환
}

void CVC::calculateAPR(const pcl::PointCloud<pcl::PointXYZI>& cloud_IN, std::vector<PointAPR>& vapr) {
  // 포인트 클라우드의 각 점에 대해 아래 계산 수행
  for (int i = 0; i < (int)cloud_IN.points.size(); ++i) {
    PointAPR par;
    // x, y 좌표를 사용하여 polar angle을 계산
    par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
    // x, y 좌표를 사용하여 range 값을 계산
    par.range = sqrt(cloud_IN.points[i].x * cloud_IN.points[i].x + cloud_IN.points[i].y * cloud_IN.points[i].y);
    // x, z 좌표를 사용하여 azimuth 값을 계산
    par.azimuth = (float)atan2(cloud_IN.points[i].z, par.range);
    
    // 계산된 range 값을 이용하여 min_range_, max_range_ 업데이트
    if (par.range < min_range_) {
      min_range_ = par.range;
    }
    if (par.range > max_range_) {
      max_range_ = par.range;
    }
    
    // 계산된 값들을 vapr 벡터에 추가
    vapr.push_back(par);
  }
  
  // length_, width_, height_ 값 계산
  length_ = int((max_range_ - min_range_) / deltaR_) + 1;
  width_  = round(360 / deltaP_);
  height_ = int(((max_azimuth_ - min_azimuth_) * 180 / M_PI) / deltaA_) + 1;
}

// 해시 테이블 구축 함수
void CVC::build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out) {
    std::vector<int> ri;  // 거리 인덱스를 저장할 배열
    std::vector<int> pi;  // 극좌표 인덱스를 저장할 배열
    std::vector<int> ai;  // 방위각 인덱스를 저장할 배열

    for (int i = 0; i < (int)vapr.size(); ++i) {
        // 현재 PointAPR의 극좌표, 거리, 방위각에 대한 인덱스 계산
        int azimuth_index = int( ( (vapr[i].azimuth - min_azimuth_) * 180 / M_PI ) / deltaA_ );
        int polar_index = int(vapr[i].polar_angle * 180 / M_PI / deltaP_);
        int range_index = int((vapr[i].range - min_range_) / deltaR_);

        // 현재 요소의 voxel_index 계산
        int voxel_index = (polar_index * (length_) + range_index) + azimuth_index * (length_ * width_);
        
        // 계산된 인덱스 정보 저장
        ri.push_back(range_index);
        pi.push_back(polar_index);
        ai.push_back(azimuth_index);

        // 해시 테이블에서 해당 voxel_index를 가진 요소 검색
        std::unordered_map<int, Voxel>::iterator it_find;
        it_find = map_out.find(voxel_index);

        // 이미 있는 키에 대한 처리
        if (it_find != map_out.end()) {
            it_find->second.index.push_back(i);  // 이미 존재하는 요소의 인덱스 추가
        }
        // 새로운 키에 대한 처리
        else {
            Voxel vox;
            vox.haspoint = true;
            vox.index.push_back(i);  // 새로운 요소 생성 및 인덱스 추가
            vox.index.swap(vox.index);
            map_out.insert(std::make_pair(voxel_index, vox));  // 해시맵에 요소 삽입
        }
    }
  //auto maxPosition = max_element(ai.begin(), ai.end());
  //auto maxPosition1 = max_element(ri.begin(), ri.end());
  //auto maxPosition2 = max_element(pi.begin(), pi.end());
}

// 이웃 점 찾기 함수
void CVC::find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex) {
  for (int z = azimuth - 1; z <= azimuth + 1; z++) {
    
    if (z < 0 || z > (height_ - 1)) {
        continue;  // 방위값이 유효 범위를 벗어나면 다음 반복으로 이동
    }

    for (int y = range - 1; y <= range + 1; y++) {
      if (y < 0 || y > (length_ - 1)) {
          continue;  // 거리값이 유효 범위를 벗어나면 다음 반복으로 이동
      }

      for (int x = polar - 1; x <= polar + 1; x++) {
          int px = x;
          if (x < 0) {
              px = width_ - 1;  // 폭값이 음수인 경우, 폭 값을 조정하여 인덱스 계산
          }
          if (x > (width_ - 1)) {
              px = 0;  // 폭값이 폭을 초과한 경우, 폭 값을 조정하여 인덱스 계산
          }
          neighborindex.push_back((px * length_ + y) + z * (length_ * width_));
          // 이웃 점의 인덱스를 계산하여 배열에 추가
      }
    }
  }
}



bool CVC::most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index) {
  std::unordered_map<int, int> histcounts;
  for (int i = 0; i < (int)values.size(); i++) {
    if (histcounts.find(values[i]) == histcounts.end()) {
      histcounts[values[i]] = 1;
    }
    else {
      histcounts[values[i]] += 1;
    }
  }
  // int max = 0, maxi;
  std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
  sort(tr.begin(),tr.end(),compare_cluster);
  for(int i = 0 ; i< (int)tr.size(); ++i){
    if(tr[i].second>10){
      cluster_index.push_back(tr[i].first);
    }
  }
  
  return true;
}

std::vector<int>  CVC::cluster(std::unordered_map<int, Voxel> &map_in,const std::vector<PointAPR>& vapr){
  int current_cluster = 0;
  //printf("Doing CVC cluster");
  std::vector<int> cluster_indices = std::vector<int>(vapr.size(), -1);

  for(int i = 0; i< (int)vapr.size(); ++i) {
    if (cluster_indices[i] != -1)
      continue;
    int azimuth_index = int((vapr[i].azimuth - min_azimuth_)*180/M_PI/deltaA_);
    int polar_index   = int(vapr[i].polar_angle*180/M_PI/deltaP_);
    int range_index   = int((vapr[i].range-min_range_)/deltaR_);
    int voxel_index   = (polar_index*(length_)+range_index)+azimuth_index*(length_)*(width_);
    
    std::unordered_map<int, Voxel>::iterator it_find;
    std::unordered_map<int, Voxel>::iterator it_find2;
    
    it_find = map_in.find(voxel_index);
    std::vector<int> neightbors;
    
    if (it_find != map_in.end()){
      std::vector<int> neighborid;
      find_neighbors(polar_index, range_index, azimuth_index, neighborid);
      for (int k =0; k<(int)neighborid.size(); ++k){
        it_find2 = map_in.find(neighborid[k]);
        if (it_find2 != map_in.end()){
          for(int j =0 ; j<(int)it_find2->second.index.size(); ++j){
            neightbors.push_back(it_find2->second.index[j]);
          }
        }
      }
    }

    neightbors.swap(neightbors);

  if(neightbors.size()>0){
    for(int j =0 ; j<(int)neightbors.size(); ++j){
      int oc = cluster_indices[i] ;
      int nc = cluster_indices[neightbors[j]];
      if (oc != -1 && nc != -1) {
        if (oc != nc)
          mergeClusters(cluster_indices, oc, nc);
        }
        else {
          if (nc != -1) {
            cluster_indices[i] = nc;
          }
          else {
            if (oc != -1) {
              cluster_indices[neightbors[j]] = oc;
            }
          }
        }
      }
    }
  
    if (cluster_indices[i] == -1) {
      current_cluster++;
      cluster_indices[i] = current_cluster;
      for(int s =0 ; s<(int)neightbors.size(); ++s) {             
        cluster_indices[neightbors[s]] = current_cluster;
      }
    }
  }
  return cluster_indices;
}

void CVC::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2) {
  for (int i = 0; i < (int)cluster_indices.size(); i++) {
    if (cluster_indices[i] == idx1) {
      cluster_indices[i] = idx2;
    }
  }
}
