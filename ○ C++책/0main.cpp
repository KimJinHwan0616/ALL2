#include <iostream>
#include <atomic>
#include <thread>

int r1 = 0, r2 = 0;
std::atomic<int> x(0), y(0);

void thread1() {
    r1 = y.load(std::memory_order_relaxed); // A
    x.store(r1, std::memory_order_relaxed); // B
}

void thread2() {
    r2 = x.load(std::memory_order_relaxed); // C
    y.store(42, std::memory_order_relaxed); // D
}
int main() {
    std::thread t1(thread1);
    std::thread t2(thread2);

    t1.join();
    t2.join();

    std::cout << "r1: " << r1 << ", r2: " << r2 << std::endl;
    std::cout << "Final x: " << x.load() << ", Final y: " << y.load() << std::endl;

    return 0;
}
