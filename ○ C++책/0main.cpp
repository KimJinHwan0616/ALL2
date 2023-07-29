#include <iostream>

// 예시를 위해 drivers::PointCloud를 간단한 구조체로 대체합니다.
// 실제로는 drivers::PointCloud가 어떤 구조를 갖고 있는지를 정의해야 합니다.
struct PointCloud {
    float x;
    float y;
    float z;
};

class SwmLidar2cameraFusionComponent {
public:
    SwmLidar2cameraFusionComponent() = default;

    // 예시를 위해 간단한 멤버 변수를 추가합니다.
    int id;
    PointCloud point_cloud;
};

int main() {
    SwmLidar2cameraFusionComponent component;

    // 기본생성자로 객체를 생성하면, 멤버 변수들의 값은 기본적으로 초기화됩니다.
    // 정수형 변수들은 0, 실수형 변수들은 0.0으로 초기화됩니다.
    std::cout << "ID: " << component.id << std::endl;
    std::cout << "Point Cloud (x, y, z): (" << component.point_cloud.x << ", "
              << component.point_cloud.y << ", " << component.point_cloud.z << ")" << std::endl;

    return 0;
}
