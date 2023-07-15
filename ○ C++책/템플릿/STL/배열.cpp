#include <iostream>
#include <vector>

using namespace std;

int main()
{
    vector<int> v1;
    v1.reserve(3);

    v1.push_back(10);
    v1.push_back(11);
    v1.push_back(12);
    v1.push_back(13);

    cout << v1.size() << endl;
    
    cout << v1[0] << endl;
    cout << v1[1] << endl;

}

