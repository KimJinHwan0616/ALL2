#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main()
{
    // vector<double> v1;

    // v1.push_back(13.5);
    // v1.push_back(14.1);
    // v1.push_back(15);

    vector<double> v1 = {1, 2, 3};

    vector<double>::iterator itr_first, itr_last, i;

    // vector<string> v1;
    // v1.push_back("안녕");
    // v1.push_back("하이");
    
    // vector<string>::iterator itr_first, itr_last, i;

    itr_first = v1.begin();
    itr_last = v1.end();

    for (i = itr_first; i != itr_last; i++) {
        cout << *i << endl;
    }
}

