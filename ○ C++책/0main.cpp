#include <iostream>
#include <chrono>

int main() {
    using namespace std::chrono;

    // 2.5초를 나타내는 duration 객체를 생성합니다.
    auto duration_seconds = seconds(2) + milliseconds(500);

    // duration 객체를 밀리초로 변환합니다.
    auto duration_milliseconds = duration_cast<milliseconds>(duration_seconds);

    // 결과를 출력합니다.
    std::cout << "2.5 seconds in milliseconds: " << duration_milliseconds.count() << " milliseconds." << std::endl;

    return 0;
}
