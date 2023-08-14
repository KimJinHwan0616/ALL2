#include <iostream>
#include <string>
#include <utility>

int main() {
    std::string* source = new std::string("Hello, source!");
    std::string* destination = new std::string("Hello, destination!");

    std::cout << "Before move: *source = " << *source << ", *destination = " << *destination << "\n";

    *destination = std::move(*source);

    std::cout << "After move: *source = " << *source << ", *destination = " << *destination << "\n";

    delete source;
    delete destination;

    return 0;
}
