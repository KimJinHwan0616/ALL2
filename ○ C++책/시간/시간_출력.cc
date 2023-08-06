#include <iostream>
#include <chrono>

class Time {
public:
    // Time 클래스의 정적 멤버 함수로, 현재 시간을 나노초 단위로 반환하는 함수입니다.
    static Time Now() {
        // 현재 시간을 가져옵니다.
        auto now = std::chrono::high_resolution_clock::now();
        
        // 시간 단위를 나노초로 변환하여 time_point를 생성합니다.
        auto nano_time_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        
        // time_point의 에포크(시간의 기준점)로부터 경과한 시간 간격을 계산합니다.
        auto epoch = nano_time_point.time_since_epoch();
        
        // 시간 간격을 나노초 단위로 변환하여 정수로 저장합니다.
        uint64_t now_nano = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
        
        // 생성자를 호출하여 Time 객체를 생성하고, 나노초 단위의 현재 시간을 저장하여 반환합니다.
        return Time(now_nano);
    }

    // Time 클래스의 생성자로, 나노초 단위의 시간을 저장하는 함수입니다.
    Time(uint64_t nano_seconds) : nano_seconds_(nano_seconds) {}

    // 나노초 값을 반환하는 멤버 함수입니다.
    uint64_t GetNanoSeconds() const {
        return nano_seconds_;
    }

private:
    uint64_t nano_seconds_; // 나노초 단위의 시간을 저장하는 변수
};

int main() {
    // Time::Now() 함수를 호출하여 현재 시간을 나노초 단위로 가져옵니다.
    Time current_time = Time::Now();

    // 가져온 현재 시간을 출력합니다.
    std::cout << "Current time in nanoseconds: " << current_time.GetNanoSeconds() << " nanoseconds." << std::endl;

    return 0;
}
