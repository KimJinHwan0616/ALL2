#include <iostream>
#include <memory>

class MyClass {
public:
    MyClass(int value) : data(value) {
        std::cout << "MyClass Constructor with value: " << data << std::endl;
    }

    ~MyClass() {
        std::cout << "MyClass Destructor for value: " << data << std::endl;
    }

    void PrintValue() {
        std::cout << "Value: " << data << std::endl;
    }

private:
    int data;
};

int main() {
    std::shared_ptr<MyClass> ptr1 = std::make_shared<MyClass>(10);

    // ptr1이 참조하는 객체의 PrintValue 호출
    ptr1->PrintValue();

    // ptr1.reset()을 호출하여 ptr1이 다른 객체나 nullptr을 가리키도록 함
    ptr1.reset();

    std::cout << "After ptr1.reset()" << std::endl;

    // ptr1.reset()을 호출했으므로 ptr1은 nullptr을 가리킴
    if (ptr1 == nullptr) {
        std::cout << "ptr1 is nullptr" << std::endl;
    }

    std::shared_ptr<MyClass> ptr2 = std::make_shared<MyClass>(20);

    // ptr2가 ptr1을 가리키도록 함
    ptr1 = ptr2;

    std::cout << "After assigning ptr2 to ptr1" << std::endl;

    // ptr1이 ptr2가 가리키는 객체의 PrintValue 호출
    ptr1->PrintValue();

    return 0;
}
