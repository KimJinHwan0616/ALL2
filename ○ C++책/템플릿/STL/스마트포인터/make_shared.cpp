#include <iostream>
#include <memory>
#include <string>

class Bicycle {
public:
    Bicycle(const std::string& owner) : owner(owner) {}

    std::string owner;
};

int main() {
    // 각자 다른 자전거를 개인적으로 소유하는 std::shared_ptr 생성
    std::shared_ptr<Bicycle> son_bike = std::make_shared<Bicycle>("아들");
    std::shared_ptr<Bicycle> daughter_bike = std::make_shared<Bicycle>("딸");

    // 아들과 딸은 각자 자신의 자전거를 사용함
    std::cout << "아들이 자전거를 빌렸습니다." << std::endl;
    std::cout << "딸이 자전거를 빌렸습니다." << std::endl;

    // 아들과 딸은 각자 자신의 자전거를 소유하고 있음
    return 0;
} // 아들과 딸이 각자 자신의 자전거를 사용하고 있으므로 메모리 자동 해제 없음