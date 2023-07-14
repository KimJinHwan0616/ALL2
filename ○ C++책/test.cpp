#include <iostream>
#include <typeinfo>

template <typename T>
struct Point {
    T x;
    T y;
};

int main()
{
    Point<float> pointFloat;

    using PointType = decltype(pointFloat);
    std::cout << typeid(PointType).name() << std::endl; 
    std::cout << "complete";

    return 0;
}
