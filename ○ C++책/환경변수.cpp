#include <iostream>
#include <cstdlib> // std::getenv를 사용하기 위한 헤더 파일

int main() {
    // getenv() //
    const char* username = std::getenv("USERNAME"); // "USERNAME" 대신에 원하는 환경 변수 이름을 넣어주세요
    
    if (username != nullptr) {
        std::cout << "사용자 이름: " << username << std::endl;
    } else {
        std::cout << "사용자 이름 환경 변수를 찾을 수 없습니다." << std::endl;
    }
    ///////////////////////////////////////////////////////
    // c_str() //
    // std::string cppString = "Hello, world!";
    
    // const char* cString = cppString.c_str();
    
    // std::cout << "C++ string: " << cppString << std::endl;
    // std::cout << "C string: " << cString << std::endl;
    ///////////////////////////////////////////////////////
    // char, char* //
    // char singleChar = 'A'; // 단일 문자를 저장하는 char 변수
    // const char* stringPointer = "Hello"; // C 스타일의 문자열을 가리키는 char 포인터

    // std::cout << "Single character: " << singleChar << std::endl;
    // std::cout << "String pointer: " << stringPointer << std::endl;

    return 0;
}
