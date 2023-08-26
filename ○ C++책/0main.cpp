#include <iostream>
using namespace std;

// explicit를 사용하지 않은 경우
class ImplicitClass {
public:
    int value;

    ImplicitClass(int v) : value(v) {}
};

void printImplicit(ImplicitClass obj) {
    cout << obj.value << endl;
}

// explicit를 사용한 경우
class ExplicitClass {
public:
    int value;

    explicit ExplicitClass(int v) : value(v) {}
};

void printExplicit(ExplicitClass obj) {
    cout << obj.value << endl;
}

int main() {
    ImplicitClass implicitObj = 42;  // 암시적 형변환 발생
    // ExplicitClass explicitObj = 42;  // 명시적 형변환 필요

    printImplicit(implicitObj);  // 암시적 형변환
    // 아래 줄은 컴파일 에러를 발생시킵니다.
    //printExplicit(42);  // 명시적 형변환 필요

    return 0;
}
