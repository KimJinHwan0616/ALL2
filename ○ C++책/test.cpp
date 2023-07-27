#include <iostream>

// 조건에 따라 코드를 활성화/비활성화하는 매크로
#define ENABLE_FEATURE_A  // Feature A를 활성화하려면 주석 해제
//#define ENABLE_FEATURE_B  // Feature B를 활성화하려면 주석 해제

int main() {
#ifdef ENABLE_FEATURE_A
    std::cout << "Feature A is enabled." << std::endl;
#endif

#ifdef ENABLE_FEATURE_B
    std::cout << "Feature B is enabled." << std::endl;
#endif

#ifndef ENABLE_FEATURE_A
    std::cout << "Feature A is disabled." << std::endl;
#endif

#ifndef ENABLE_FEATURE_B
    std::cout << "Feature B is disabled." << std::endl;
#endif

    return 0;
}