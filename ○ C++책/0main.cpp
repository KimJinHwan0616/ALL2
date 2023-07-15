#include <iostream>
#include <vector>
#include <memory>

using namespace std;

const std::size_t kDefaultReservePointNum = 50000;

struct PointIndices {
  PointIndices() { indices.reserve(kDefaultReservePointNum); }

  vector<int> indices;

  typedef shared_ptr<PointIndices> Ptr;
  typedef shared_ptr<const PointIndices> ConstPtr;
};

int main()
{
    PointIndices pi;
    cout << pi.indices.capacity();
}

