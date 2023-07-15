#include <iostream>
#include <vector>

using namespace std;

int main()
{
    vector<int> v1;

    v1.push_back(10);
    v1.push_back(11);

    cout << (int) v1.size() << endl;
    cout << v1[0] << endl;
    cout << v1[1] << endl;

}

