#include <iostream>
#include <unordered_map>
#include <string>

int main() {
    std::unordered_multimap<std::string, std::string> itemsByCategory;

    // 물건을 카테고리에 추가
    itemsByCategory.emplace("과일", "사과");
    itemsByCategory.emplace("음료", "우유");
    itemsByCategory.emplace("과일", "딸기");
    itemsByCategory.emplace("빵류", "식빵");
    itemsByCategory.emplace("음료", "콜라");

    // 각 카테고리별로 물건 목록 출력
    std::cout << "카테고리별 물건 목록:" << std::endl;
    for (const auto& pair : itemsByCategory) {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }

    // 특정 카테고리의 물건 검색
    std::string categoryToSearch = "과일";
    auto range = itemsByCategory.equal_range(categoryToSearch);
    std::cout << categoryToSearch << " 카테고리의 물건 목록:" << std::endl;
    for (auto it = range.first; it != range.second; ++it) {
        std::cout << "- " << it->second << std::endl;
    }

    return 0;
}
