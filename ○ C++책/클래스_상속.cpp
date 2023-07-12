#include <iostream>
#include <string.h>

using namespace std;

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

int main()
{
    Fruit myFruit;

    myFruit.SetPrice(150);
    cout << myFruit.GetPrice() << endl;

    myFruit.SetFarName("성안당 농장");
    cout << myFruit.GetFarName() << endl;
}

