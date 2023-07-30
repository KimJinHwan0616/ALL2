#include <Eigen/Core>
#include <iostream>

int main() {
    Eigen::Affine3d transform;

    // 단위 아핀 변환으로 초기화
    transform.setIdentity();

    std::cout << "Transform matrix:\n" << transform.matrix() << std::endl;

    return 0;
}
