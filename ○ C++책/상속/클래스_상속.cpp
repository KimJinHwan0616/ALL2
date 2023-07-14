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
    void SetFarName(const char *farm) {
        strcpy(farm_name, farm);
    }
    char *GetFarName() {
        return farm_name;
    }

private:
    char farm_name[50];
};

// 3
int main()
{
    Food *pFood[2];
    Fruit myFruit;
    
    // myFruit.SetPrice(150);
    // cout << myFruit.GetPrice() << endl;

    // myFruit.SetFarName("성안당 농장");
    // cout << myFruit.GetFarName() << endl;

    pFood[0] = &myFruit;
}

