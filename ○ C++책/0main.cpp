#include <iostream>
#include <unordered_map>
#include <unordered_set>

int main() {
    std::unordered_map<int, std::string> myMap;

    myMap[1] = "one";
    myMap[2] = "two";
    myMap[3] = "three";
    myMap[2] = "second"; // 기존 값 대체

    std::cout << "std::unordered_map:" << std::endl;
    for (const auto& pair : myMap) {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }

    std::unordered_multimap<int, std::string> myMultiMap;

    myMultiMap.emplace(1, "first");
    myMultiMap.emplace(2, "second");
    myMultiMap.emplace(2, "another second"); // 같은 키에 여러 값 삽입
    myMultiMap.emplace(3, "third");

    std::cout << "\nstd::unordered_multimap:" << std::endl;
    for (const auto& pair : myMultiMap) {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }

    return 0;
}
