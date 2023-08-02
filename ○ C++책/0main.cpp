#include <iostream>
#include <vector>

// 포인트 클라우드를 나타내는 PointT 구조체 정의
struct PointT {
  float x;
  float y;
  float z;
};

// 포인트 클라우드 클래스 정의
class PointCloud {
 public:
  // 포인트들을 저장하는 벡터
  std::vector<PointT> points_;

  // 포인트 클라우드의 가로와 세로 크기
  size_t width_ = 0;
  size_t height_ = 0;

  // 생성자
  PointCloud() = default;

  // CopyPointCloudExclude 함수 선언
  template <typename IndexType>
  inline void CopyPointCloudExclude(const PointCloud& rhs, const std::vector<IndexType>& indices);

  // IsOrganized 함수 선언
  bool IsOrganized() const { return height_ > 1; }
};

// CopyPointCloudExclude 함수 정의
template <typename IndexType>
inline void PointCloud::CopyPointCloudExclude(const PointCloud& rhs, const std::vector<IndexType>& indices) {
  width_ = rhs.width_;
  height_ = rhs.height_;
  points_.clear();

  // rhs의 크기와 같은 크기의 false로 초기화된 mask 벡터 생성
  std::vector<bool> mask(rhs.points_.size(), false);

  // 제외할 인덱스들을 true로 설정하여 mask 생성
  for (size_t i = 0; i < indices.size(); ++i) {
    if (indices[i] < rhs.points_.size()) {
      mask[indices[i]] = true;
    }
  }

  // mask에 따라 포인트를 복사하여 points_ 벡터에 저장
  for (size_t i = 0; i < rhs.points_.size(); ++i) {
    if (!mask[i]) {
      points_.push_back(rhs.points_[i]);
    }
  }
}

int main() {
  // 원본 포인트 클라우드 생성
  PointCloud original_cloud;
  original_cloud.width_ = 3;
  original_cloud.height_ = 2;
  original_cloud.points_ = { 
    {1.0f, 2.0f, 3.0f},
    {4.0f, 5.0f, 6.0f},
    {7.0f, 8.0f, 9.0f},
    {10.0f, 11.0f, 12.0f},
    {13.0f, 14.0f, 15.0f},
    {16.0f, 17.0f, 18.0f}
  };

  // 제외할 인덱스 배열 생성
  std::vector<size_t> exclude_indices = {1, 3, 5};

  // CopyPointCloudExclude 함수 호출하여 제외된 포인트들을 포함한 새로운 포인트 클라우드 생성
  PointCloud excluded_cloud;
  excluded_cloud.CopyPointCloudExclude(original_cloud, exclude_indices);

  // 결과 출력
  std::cout << "Original Cloud:" << std::endl;
  for (size_t i = 0; i < original_cloud.points_.size(); ++i) {
    std::cout << "(" << original_cloud.points_[i].x << ", " << original_cloud.points_[i].y << ", " << original_cloud.points_[i].z << ")" << std::endl;
  }

  std::cout << "Excluded Cloud:" << std::endl;
  for (size_t i = 0; i < excluded_cloud.points_.size(); ++i) {
    std::cout << "(" << excluded_cloud.points_[i].x << ", " << excluded_cloud.points_[i].y << ", " << excluded_cloud.points_[i].z << ")" << std::endl;
  }

  return 0;
}
