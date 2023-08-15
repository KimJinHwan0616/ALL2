#include <iostream>
#include <string>

int main() {
    std::string cppString = "Hello, World!";
    const char* cString = cppString.c_str();

    printf("%s\n", cString);

    return 0;
}
