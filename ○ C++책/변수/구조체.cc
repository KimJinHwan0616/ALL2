#include <iostream>
#include <string>

using namespace std;

struct Address {
    string city;
    string street;
    int zipCode;
};

struct Person {
    string name;
    int age;
    Address* address; // 포인터 멤버 변수
};

int main() {
    // Address 구조체 객체 생성
    Address address1;
    address1.city = "Seoul";
    address1.street = "123 Main St";
    address1.zipCode = 12345;

    // Person 구조체 객체 생성
    Person person1;
    person1.name = "John";
    person1.age = 30;
    person1.address = &address1; // address1의 주소를 person1의 address 멤버에 할당

    // 두 번째 Person 객체를 생성하고, 이 객체의 주소를 가리키는 포인터 선언
    Person person2;
    person2.name = "Jane Smith";
    person2.age = 25;
    person2.address = new Address(); // 동적으로 Address 구조체 객체 할당
    person2.address->city = "New York";
    person2.address->street = "456 Oak Ave";
    person2.address->zipCode = 67890;

    // person1 정보 출력
    cout << "Name: " << person1.name << endl;
    cout << "Age: " << person1.age << endl;
    cout << "Address: " << person1.address->street << ", " << person1.address->city << ", " << person1.address->zipCode << endl;

    // person2 정보 출력
    cout << "Name: " << person2.name << endl;
    cout << "Age: " << person2.age << endl;
    cout << "Address: " << person2.address->street << ", " << person2.address->city << ", " << person2.address->zipCode << endl;

    // 동적으로 할당한 구조체 메모리 해제
    delete person2.address;

    return 0;
}
