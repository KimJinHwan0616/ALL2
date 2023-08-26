#include <iostream>
#include <string>
using namespace std;

class PartyInvitation {
public:
    string guestName;

    // 초대장 생성자
    PartyInvitation(string name) : guestName(name) {}

    // 복사 생성자를 delete로 설정하여 복사 금지
    PartyInvitation(const PartyInvitation &other) = delete;
};

int main() {
    PartyInvitation friend1("Alice");
    PartyInvitation friend2("Bob");

    // 아래 줄은 컴파일 에러를 발생시킵니다.
    // PartyInvitation invitationCopy = friend1;

    return 0;
}
