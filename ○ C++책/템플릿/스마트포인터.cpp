#include <iostream>
#include <memory>

using namespace std;

class Person {
public:
    Person(const string &name, int age);  // 생성자
    ~Person() { cout << "생성자 소멸" << endl; }

    void ShowPersonInfo();

private:
    string name_;
    int age_;
};

Person::Person(const string &name, int age)
{
    name_ = name;
    age_ = age;
    cout << "생성자 호출" << endl;
}

void Person::ShowPersonInfo() {
    cout << name_ << age_ << endl;
}

////////////////////////////////////

// unique_ptr
// int main() {
//     unique_ptr<Person> hong = make_unique<Person>("길동", 29);
//     hong -> ShowPersonInfo();
// }

// shared_ptr
int main() {
    shared_ptr<Person> hong = make_shared<Person>("길동", 29);
    cout << "현재 소유자 수 : " << hong.use_count() << endl;

    auto han = hong;
    cout << "현재 소유자 수 : " << hong.use_count() << endl;

    han.reset();
    cout << "현재 소유자 수 : " << hong.use_count() << endl;
}

