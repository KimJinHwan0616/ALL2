#include <iostream>
#include <string.h>

using namespace std;

class Student 
{
public:
    void SetID(const int num) { id = num; }
    void SetName(const char *str) { strcpy(name, str); }

    int GetID() { return id; };
    char *GetName() const { return (char *)name; }

private:
    int id;
    char name[30];
};


int main()
{

}

