#include <array>
#include <iostream>

int main() {
    std::array<int, 5> myArray = {1, 2, 3, 4, 5};

    for (int i = 0; i < myArray.size(); ++i) {
        std::cout << myArray[i] << " ";
    }

    std::cout << "\nFront: " << myArray.front();
    std::cout << "\nBack: " << myArray.back() << std::endl;

    return 0;
}
