#include <iostream>
#include <memory>
#include <string>

class Bicycle {
public:
    Bicycle(const std::string& owner) : owner(owner) {}

    std::string owner;
};

int main() {
    // 가족 구성원들이 자전거를 공유할 수 있는 std::shared_ptr 생성
    std::shared_ptr<Bicycle> shared_bicycle = std::make_shared<Bicycle>("가족");

    // 아들이 자전거를 빌림
    {
        std::shared_ptr<Bicycle> son_bike = shared_bicycle;
        std::cout << "아들이 자전거를 빌렸습니다." << std::endl;
    } // 아들이 자전거를 반납, 여전히 가족 구성원들이 공유 중

    // 딸이 자전거를 빌림
    {
        std::shared_ptr<Bicycle> daughter_bike = shared_bicycle;
        std::cout << "딸이 자전거를 빌렸습니다." << std::endl;
    } // 딸이 자전거를 반납, 여전히 가족 구성원들이 공유 중

    // 가족 구성원들이 계속해서 자전거를 공유 중
    return 0;
} // 모든 가족 구성원이 자전거 사용을 중단하고, 자전거 메모리 자동 해제
