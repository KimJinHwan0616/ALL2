#include <iostream>
#include <string>
#include <functional>  // for std::hash

int main() {
    std::string key = "Hello, World!";

    // 해시 함수를 사용하여 문자열을 해싱하여 해시 코드를 생성
    std::size_t hashCode = std::hash<std::string>{}(key);

    std::cout << "Original Key: " << key << std::endl;
    std::cout << "Hash Code: " << hashCode << std::endl;
    std::cout << "Type of hashCode: " << typeid(hashCode).name() << std::endl;

    return 0;
}
