#include <iostream>

class Bread {
private:
    int quantity;

public:
    Bread(int quantity) {
        this->quantity = quantity;
    }

    int getQuantity() {
        return this->quantity;
    }
};

int main() {
    Bread bread1(5);  // Bread 객체 생성, 빵의 수량은 5개
    Bread bread2(10); // Bread 객체 생성, 빵의 수량은 10개

    std::cout << "Bread 1 quantity: " << bread1.getQuantity() << std::endl;
    std::cout << "Bread 2 quantity: " << bread2.getQuantity() << std::endl;

    return 0;
}
