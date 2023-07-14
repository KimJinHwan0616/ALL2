#include <iostream>

using namespace std;

// 1
class Food {
public:
    virtual void SetPrice(int myprice) = 0;   // 순수 가상함수

    int GetPrice() {
        return price;
    }

protected:
    int price;
};

// 2
class Fruit : public Food {
public:
    void SetPrice(int myprice) override {    // 오버라이딩
        price = myprice - 20;
    }
};

// 3
class Fish : public Food {
public:
    void SetPrice(int myprice) override {
        price = myprice / 2;
    }
};

// 4
int main()
{
    Food *pFood;
    Fruit myFruit;
    Fish myFish;

    pFood = &myFruit;
    pFood -> SetPrice(100);
    cout << pFood -> GetPrice() << endl;

    pFood = &myFish;
    pFood -> SetPrice(100);
    cout << pFood -> GetPrice() << endl;
}

