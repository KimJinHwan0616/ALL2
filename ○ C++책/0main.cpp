#include <iostream>
#include <atomic>
#include <thread>

int main() {
    std::atomic<int> value = 0;  // 정수형의 std::atomic 변수 생성 및 초기화

    std::cout << "Initial value: " << value.load() << std::endl;

    std::thread t1([&]() {
        value.store(10, std::memory_order_relaxed);  // 값을 10으로 저장
    });

    std::thread t2([&]() {
        value.store(20, std::memory_order_relaxed);  // 값을 20으로 저장
    });

    t1.join();
    t2.join();

    std::cout << "Final value: " << value.load() << std::endl;

    return 0;
}
