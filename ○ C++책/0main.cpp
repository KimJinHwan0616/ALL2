#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<int> numbers = {1, 2, 3, 4, 5, 3, 6};

    // 조건 함수: 3과 같은 요소를 제거하기 위한 조건자
    auto condition = [](int num) {
        return num == 3;
    };

    // remove_if 함수 사용
    auto newEnd = std::remove_if(numbers.begin(), numbers.end(), condition);

    // 제거된 요소 출력
    std::cout << "Removed elements: ";
    for (auto it = newEnd; it != numbers.end(); ++it) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;

    // 남은 요소 출력
    std::cout << "Remaining elements: ";
    for (auto it = numbers.begin(); it != newEnd; ++it) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;

    return 0;
}
