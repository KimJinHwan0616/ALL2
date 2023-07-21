#include <iostream>
#include <atomic>
#include <thread>

std::atomic<uint32_t> seq_num_(0);

void incrementSequenceNumber() {
    for (int i = 0; i < 10; ++i) {
        uint32_t seq_num = seq_num_.fetch_add(1);
        std::cout << "Thread ID: " << std::this_thread::get_id() << ", Sequence Number: " << seq_num << std::endl;
    }
}

int main() {
    std::thread thread1(incrementSequenceNumber);
    std::thread thread2(incrementSequenceNumber);

    thread1.join();
    thread2.join();

    return 0;
}
