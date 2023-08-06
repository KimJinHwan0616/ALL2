#include <iostream>
#include <chrono>

using namespace std::chrono;

int main() {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Some time-consuming task here...

    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = end_time - start_time;

    // duration.count()를 통해 실행 시간을 나노초 단위로 얻을 수 있습니다.
    std::cout << "Elapsed time: " << duration.count() << " nanoseconds." << std::endl;

    return 0;
}
