#include <iostream>
#include <functional>

// 두 수를 더하는 콜백 함수 타입
using AddCallback = std::function<int(int, int)>;

// 두 수를 더하는 함수
int Add(int a, int b) {
    return a + b;
}

// 두 수를 더하는 콜백 함수 실행
int PerformAdd(AddCallback callback, int x, int y) {
    return callback(x, y);
}

int main() {
    // 두 수를 더하는 콜백 함수를 전달하며 실행
    int result = PerformAdd(Add, 3, 5);
    std::cout << "Sum: " << result << std::endl;
    return 0;
}
