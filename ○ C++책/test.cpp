#include <iostream>

// 인수가 있고 반환값이 있는 함수
int multiplyNumbers(int x, int y) {
    return x * y;
}

int main() {
    int num1 = 4;
    int num2 = 6;
    int result = multiplyNumbers(num1, num2); // 함수 호출
    std::cout << "Result: " << result << std::endl;
    return num1;
}
