#include <iostream>
#include <map>
#include <string>

int main() {
    std::map<std::string, int> dict;

    dict["수학"] = 100;
    dict["영어"] = 70;
    dict["국어"] = 50;

    std::string subject_to_find = "영어";
    auto range = dict.equal_range(subject_to_find);

    std::cout << "Scores for the subject " << ":" << subject_to_find << std::endl;
    for (auto it = range.first; it != range.second; ++it) {
        std::cout << "Score: " << it->second << std::endl;
    }

    return 0;
}
