#include <iostream>

int main() {
    // 1. 포인터 선언
    int* ptr; 

    // 2. 포인터 초기화
    int value = 42; 
    ptr = &value;

    *ptr = 10;

    std::cout << "value: " << value << std::endl; // 출력 결과: 10
    std::cout << "*ptr: " << *ptr << std::endl;   // 출력 결과: 10
    /////////////////////////////////////////////////////
    // 포인터 크기
    // int* ptr_int;
    // float* ptr_float;

    // std::cout << "Size of int* pointer: " << sizeof(ptr_int) << " bytes" << std::endl;
    // std::cout << "Size of float* pointer: " << sizeof(ptr_float) << " bytes" << std::endl;
    /////////////////////////////////////////////////////
    // NULL 포인터
    // int* ptr = nullptr; // 초기화

    // std::cout << "Value of ptr: " << ptr << std::endl; // ptr의 값, 즉 주소값 출력
    // std::cout << "Address of ptr: " << &ptr << std::endl; // ptr 변수 자체의 주소 출력

    return 0;
}
