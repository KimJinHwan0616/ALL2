#include <iostream>
#include <vector>

// 아이템(Item) 클래스
class Item {
public:
    Item(std::string name) : name_(name) {}
    std::string getName() const { return name_; }

private:
    std::string name_;
};

// 박스(Box) 클래스
class Box {
public:
    // Default constructor
    Box() {}

    // Constructor taking an initializer list of Item objects
    Box(std::initializer_list<Item> items) : items_(items) {}

    void CopyItems(const Box& rhs);
    void PrintItems() const;

private:
    std::vector<Item> items_;
};

// 아이템 복사 함수 정의
void Box::CopyItems(const Box& rhs) {
    items_ = rhs.items_;
}

// 아이템 출력 함수 정의
void Box::PrintItems() const {
    for (const auto& item : items_) {
        std::cout << item.getName() << " ";
    }
    std::cout << std::endl;
}

int main() {
    // 첫 번째 박스 생성
    Box box1{ Item("Item1"), Item("Item2"), Item("Item3") };

    // 두 번째 박스 생성
    Box box2{ Item("ItemA"), Item("ItemB") };

    // 첫 번째 박스의 아이템들을 두 번째 박스로 복사
    box2.CopyItems(box1);

    // 두 번째 박스의 아이템 출력
    box2.PrintItems();

    return 0;
}
