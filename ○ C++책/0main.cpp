#include <iostream>
#include <cstring> // memcpy 함수를 사용하기 위해 필요한 헤더

int main() {
    char source[] = "Hello, world!"; // 소스 메모리 영역
    char target[20]; // 타겟 메모리 영역

    // source에서 target으로 데이터 복사
    std::memcpy(target, source, sizeof(source));

    std::cout << "Source: " << source << std::endl;
    std::cout << "Target: " << target << std::endl;

    return 0;
}
