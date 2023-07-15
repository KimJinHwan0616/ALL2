#include <iostream>

class OrangeBox {
public:
    OrangeBox();    // 생성자 선언
    ~OrangeBox();   // 소멸자 선언

    void Add(int addorangeBox);     // 오렌지를 넣는다.
    void Del(int delorangeBox);     // 오렌지를 꺼낸다.
    void Empty();       // 상자를 비운다.
    int GetTotal() {    // 오렌지의 총 개수를 반환한다.
        return total;
    }

private:
    int total;
};

// 생성자 정의
OrangeBox::OrangeBox()    
{
    total = 0;      // 멤버변수 초기화
}

// 소멸자 정의
OrangeBox::~OrangeBox() 
{
    std::cout << "오렌지 상자 오브젝트 임무 완료";
}


// 멤버 함수

void OrangeBox::Add(int addorange) {
    total += addorange;
    if (total > 100) total = 100;
}

void OrangeBox::Del(int delorange) {
    total -= delorange;
    if (total < 0) Empty();
}

void OrangeBox::Empty() {
    total = 0;
}

int main()
{
    OrangeBox myOrangeBox;      // 인스턴스 생성 OR 생성자 호출
    myOrangeBox.Empty();     // 제일 먼저 상자를 비운다.

    myOrangeBox.Add(5);
    myOrangeBox.Del(2);

    // myOrangeBox.total = 10;

    std::cout << "상자 속의 오렌지 : " << myOrangeBox.GetTotal() << std::endl;
}

