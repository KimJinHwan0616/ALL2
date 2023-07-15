#include <iostream>

using namespace std;

template<typename T>
T GetMax(T a, T b)  
{
    T buf;

    if (a<b) buf = 0;
    else buf = b;

    return buf;
}   

int main()
{
    int n1 = 1, n2 = 3, ret1;
    double r1 = 3.5, r2 = 1.5, ret2;

    ret1 = GetMax(n1, n2);
    ret2 = GetMax(r1, r2);

    cout << ret1 << endl;
    cout << ret2 << endl;
}

