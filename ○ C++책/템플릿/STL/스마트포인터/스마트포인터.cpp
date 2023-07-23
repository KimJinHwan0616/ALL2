#include <iostream>
#include <memory>

using namespace std;

int main() {
    auto pdata = std::make_shared<double>(999.0);
    std::shared_ptr<double> pdata2 {pdata};
}

