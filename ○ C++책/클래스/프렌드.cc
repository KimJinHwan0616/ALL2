#include <iostream>

class Cat;  // 고양이 클래스 선언

class Dog {
private:
    int age;
    friend class Cat;  // Cat 클래스가 Dog 클래스의 private 멤버에 접근할 수 있도록 선언

public:
    Dog(int _age) : age(_age) {}
};

class Cat {
public:
    void PrintDogAge(const Dog& dog) {
        // Cat 클래스는 Dog 클래스의 private 멤버에 접근 가능
        std::cout << "The dog's age is: " << dog.age << std::endl;
    }
};

int main() {
    Dog myDog(3);  // Dog 객체 생성, 나이 3살
    Cat myCat;

    myCat.PrintDogAge(myDog);  // Cat 클래스의 함수를 통해 Dog의 private 멤버에 접근

    return 0;
}
