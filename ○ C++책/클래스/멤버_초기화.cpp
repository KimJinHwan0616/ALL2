#include <iostream>
#include <vector>
#include <memory>

using namespace std;

class Date {
public:
    Date(int y, int m, int d) {
        year_ = y;
        month_ = m;
        day_ = d;
    }
private:
    int year_, month_, day_;
};

class Food {
public:
    Food();
    void SetPrice(int n) {
        price_ = n;
    }

private:
    int price_;

    Date LimitDate;
};

Food::Food() : LimitDate(2023, 7, 18), price_(100)
{

}

int main()
{

}

