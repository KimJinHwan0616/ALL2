#include <iostream>

int main() {
    int x = 5;
    int y = 10;

    // 외부 변수 x를 참조로 캡처하고 매개변수 b를 받아 덧셈 수행
    auto add_and_print = [&x](int a, int b) -> int {
        int result = a + b + x;
        // return result;
        std::cout << "Result: " << result << std::endl;
    };

    add_and_print(3, y);

    return 0;
}
