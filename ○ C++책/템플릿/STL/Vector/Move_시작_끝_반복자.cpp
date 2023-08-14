#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>

int main() {
    std::vector<std::string> local_box1 = {"apple", "banana"};
    std::vector<std::string> box2 = {"orange"};

    std::cout << "Before moving: local_box1: ";
    for (const std::string& item : local_box1) {
        std::cout << item << " ";
    }
    std::cout << "\n";

    std::cout << "Before moving: box2: ";
    for (const std::string& item : box2) {
        std::cout << item << " ";
    }
    std::cout << "\n";

    std::move(local_box1.begin(), local_box1.end(), std::back_inserter(box2));

    std::cout << "After moving: local_box1: "; // local_box1 is now empty
    for (const std::string& item : local_box1) {
        std::cout << item << " ";
    }
    std::cout << "\n";

    std::cout << "After moving: box2: "; // box2 now contains all items
    for (const std::string& item : box2) {
        std::cout << item << " ";
    }
    std::cout << "\n";

    return 0;
}
