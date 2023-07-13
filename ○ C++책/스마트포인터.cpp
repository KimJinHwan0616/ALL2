#include <iostream>
#include <memory>

using namespace std;

class Test {
public:
    Test() { cout << "생성자" << endl; }
    ~Test() { cout << "소멸자" << endl; }
};

int main() {
    cout << "main 함수 시작" << endl;
    unique_ptr<Test> a(new Test);
    cout << "main 함수 종료" << endl;
}