#include <iostream>
#include <vector>
#include <memory>

using namespace std;

class Dog {
public:
    Dog(int a) { age_ = a; }    // 

    int getAge() const;     // 상수멤버변수
    void setAge(int a) { age_ = a; }
    void view() const { cout << "안녕"; }    // 상수멤버변수

private:
    int age_;
};

int Dog::getAge() const
{
    view();
    // return ++age_;     // 오류: 상수멤버변수
    return age_;
}

int main()
{
    const Dog Happy(5);
    // Happy.setAge(7);    // 오류: 
    cout << Happy.getAge();
}

