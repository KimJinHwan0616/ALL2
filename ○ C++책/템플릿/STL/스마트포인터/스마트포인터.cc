#include <iostream>
#include <memory>
#include <string>

class Fruit {
public:
    virtual void print() {
        std::cout << "This is a Fruit." << std::endl;
    }

    virtual ~Fruit() {}
};

class Apple : public Fruit {
public:
    void print() override {
        std::cout << "This is an Apple." << std::endl;
    }
};

class Orange : public Fruit {
public:
    void print() override {
        std::cout << "This is an Orange." << std::endl;
    }
};

int main() {
    // 스마트 포인터를 사용하여 Fruit 클래스 타입으로 Apple 객체를 가리키기
    std::shared_ptr<Fruit> fruitPtr = std::make_shared<Apple>();

    // 가리키고 있는 객체의 실제 타입에 따라 적절한 함수 호출 (다형성)
    fruitPtr->print();

    // 스마트 포인터가 자동으로 메모리를 관리하므로 수동으로 삭제할 필요가 없음
    // 스마트 포인터가 범위를 벗어나면 메모리에서 자동으로 파괴됨

    return 0;
}
