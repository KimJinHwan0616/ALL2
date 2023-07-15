#include <iostream>

using namespace std;

template <typename T>
class Rectangle 
{
public:
    Rectangle (T n1, T n2, T n3, T n4) {
        left = n1 ;
        top = n2;
        width = n3;
        height = n4;
    }

    T GetRight() {
        return left + width;
    }

    T GetBottom() {
        return top + height;
    }

private:
    T left, top, width, height;
};

int main()
{
    Rectangle<int> rect1(50, 50, 100, 100);
    cout << rect1.GetRight() << endl;

    Rectangle<double> rect2(50.5, 50.5, 100.1, 100.1);
    cout << rect2.GetRight() ;
}

