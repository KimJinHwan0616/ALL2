#include <iostream>
#include <string.h>

using namespace std;

// 1
class Food {
public:
    virtual void SetPrice(int myprice) {
        price = myprice;
    }

    int GetPrice() {
        return price;
    }

private:
    int price;
};

// 2
class Fruit : public Food {
public:
    void Setprice(int myprice) {
        // Food::SetPrice(myprice - 20);
        myprice - 20;
    }
};

// 3
int main()
{
    Food *pFood;
    Fruit myFruit;

    pFood = &myFruit;
    pFood -> SetPrice(100);
    cout << pFood -> GetPrice() ;
}

