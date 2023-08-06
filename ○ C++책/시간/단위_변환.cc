#include <iostream>
#include <chrono>

int main() {
    using namespace std::chrono;

    // 현재 시간을 시스템 시계인 high_resolution_clock으로 가져옵니다.
    auto now = high_resolution_clock::now();

    // 시간 단위를 분으로 변환합니다.
    auto now_in_minutes = time_point_cast<minutes>(now);

    // 변환된 시간 스탬프를 출력합니다.
    auto minutes_count = now_in_minutes.time_since_epoch().count();
    std::cout << "Current time in minutes: " << minutes_count << " minutes." << std::endl;

    return 0;
}
