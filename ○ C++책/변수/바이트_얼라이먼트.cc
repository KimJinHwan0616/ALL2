#include <iostream>

// 패딩을 적용한 클래스 정의
class MyStruct {
public:
    int num;      // 4바이트
    char letter;  // 1바이트
    double value; // 8바이트
};

int main() {
    // 클래스 인스턴스 생성
    MyStruct myVar;

    // 클래스 멤버들의 주소 출력
    std::cout << "Address of num: " << &myVar.num << std::endl;
    std::cout << "Address of letter: " << static_cast<void*>(&myVar.letter) << std::endl;
    std::cout << "Address of value: " << &myVar.value << std::endl;

    // 클래스 크기 출력
    std::cout << "Size of MyStruct: " << sizeof(MyStruct) << " bytes" << std::endl;

    return 0;
}
