#ifndef CVC_CLUSTER_H
#define CVC_CLUSTER_H

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>


//C++
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <algorithm>
#include <limits>

template<typename T> 
std::string toString(const T& t) {
	std::ostringstream oss;
	oss << t;
	return oss.str();
}


struct PointAPR{
   float azimuth;
   float polar_angle;
   float range;
};

struct Voxel{
   bool haspoint = false;
   int cluster = -1;
   std::vector<int> index;
};



class CVC {
public:
    // 기본 생성자
    CVC() = default;
    
    // 매개변수를 받는 생성자
    CVC(std::vector<float>& param) {
        // 매개변수가 3개가 아닌 경우 오류 출력 후 프로그램 종료
        if (param.size() != 3) {
            printf("Param number is not correct!");
            std::abort();
        }
        // 매개변수 값을 각 멤버 변수에 할당
        deltaA_ = param[0];
        deltaR_ = param[1];
        deltaP_ = param[2];
    }

    // 소멸자
    ~CVC() {}

    // APR 계산 함수
    void calculateAPR(const pcl::PointCloud<pcl::PointXYZI>& cloud_IN, std::vector<PointAPR>& vapr);

    // 해시 테이블 구축 함수
    void build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel>& map_out);

    // 이웃 점 찾기 함수
    void find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex);

    // 가장 빈번한 값 찾기 함수
    bool most_frequent_value(std::vector<int> values, std::vector<int>& cluster_index);

    // 클러스터 병합 함수
    void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);

    // 클러스터링 함수
    std::vector<int> cluster(std::unordered_map<int, Voxel>& map_in, const std::vector<PointAPR>& vapr);

    // 주요 기능 실행 함수
    void process();

    float min_range() { return min_range_; }
    float max_range() { return max_range_; }

    float min_azimuth() { return min_azimuth_; }
    float max_azimuth() { return max_azimuth_; }

    float max_polar() { return max_polar_; }
    float min_polar() { return min_polar_; }

    int length() { return length_; }
    int width() { return width_; }
    int height() { return height_; }

private:
    // 점군 데이터의 최소/최대 거리와 방위각
    float min_range_ = std::numeric_limits<float>::max();
    float max_range_ = std::numeric_limits<float>::min();
    float min_azimuth_ = 360 * M_PI / 180;
    float max_azimuth_ = 0 * M_PI / 180;
    float min_polar_ = 360 * M_PI / 180;
    float max_polar_ = 0 * M_PI / 180;

    // 클러스터링에 사용되는 파라미터 값
    float deltaA_ = 0;
    float deltaR_ = 0;
    float deltaP_ = 0;

    // 클래스 내에서 사용되는 길이, 너비, 높이 값
    int length_ = 0;
    int width_ = 0;
    int height_ = 0;
};


#endif
