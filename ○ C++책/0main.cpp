#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main() {
    // 3차원 벡터를 사용하여 이동 변환 생성
    Eigen::Vector3d translation_vector(1.0, 2.0, 3.0);
    Eigen::Translation3d translation(translation_vector);

    // 이동 변환 정보 출력
    std::cout << "Translation3d Example:" << std::endl;
    std::cout << "Translation vector: " << translation_vector.transpose() << std::endl;
    std::cout << "Translation matrix:" << std::endl << translation.matrix() << std::endl;

    // 3차원 벡터에 이동 변환 적용
    Eigen::Vector3d point(0.0, 0.0, 0.0);
    Eigen::Vector3d transformed_point = translation * point;

    // 변환된 점 출력
    std::cout << "Original point: " << point.transpose() << std::endl;
    std::cout << "Transformed point: " << transformed_point.transpose() << std::endl;

    return 0;
}
