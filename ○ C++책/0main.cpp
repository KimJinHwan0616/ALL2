#include <iostream>
#include <vector>
#include <cmath> // cmath 헤더를 포함시키면 M_PI를 사용할 수 있습니다.

struct PointAPR {
    double azimuth;
    double polar_angle;
    double range;
};

int main() {
    std::vector<PointAPR> vapr = {
        {0.785398, 0.523599, 10.0},   // 예제 데이터 1
        {1.5708, 1.0472, 20.0}        // 예제 데이터 2
    };

    double min_azimuth_ = 0.0;
    double deltaA_ = 0.1;
    double deltaP_ = 0.1;
    double min_range_ = 0.0;
    double deltaR_ = 5.0;
    int length_ = 10;
    int width_ = 5;

    for (int i = 0; i < (int)vapr.size(); ++i) {
        int azimuth_index = int(((vapr[i].azimuth - min_azimuth_) * 180 / M_PI) / deltaA_);
        int polar_index = int(vapr[i].polar_angle * 180 / M_PI / deltaP_);
        int range_index = int((vapr[i].range - min_range_) / deltaR_);

        int voxel_index = (polar_index * (length_) + range_index) + azimuth_index * (length_ * width_);

        std::cout << "Data " << i + 1 << " - Voxel Index: " << voxel_index << std::endl;
    }

    return 0;
}
