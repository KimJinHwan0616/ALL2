#include <iostream>
#include <string>

int main() {
    const char* data_ = "안녕하세요";
    constexpr uint8_t ID_SIZE = 3;
    
    std::string sub_string(data_, ID_SIZE);
    
    std::cout << "하위 문자열: " << sub_string << std::endl;  // 출력: "안녕하"
    
    return 0;
}
