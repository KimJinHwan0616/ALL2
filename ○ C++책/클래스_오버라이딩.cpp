#include <iostream>
#include <string.h>

using namespace std;

// 1
class Food {
public:
    void SetPrice(int myprice) {
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
    void SetPrice(int myprice) {
        Food::SetPrice(myprice - 20);
    }
};

// 3
int main()
{
    Food myFood;
    Fruit myFruit;

    myFood.SetPrice(100);
    myFruit.SetPrice(100);

    cout << myFood.GetPrice() << endl;
    cout << myFruit.GetPrice() << endl;
}

