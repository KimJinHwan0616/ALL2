#include <iostream>
#include <vector>
#include <memory>

using namespace std;

int main()
{
// 	int* ptr_int = new int;
// 	*ptr_int = 100;
	
	// shared_ptr<int> ptr_int = make_shared<int>(100);

	// double* ptr_double = new double;
	// *ptr_double = 100.123;
	
	// cout << "int형 숫자의 값은 " << *ptr_int << "입니다." << endl;
	// cout << "int형 숫자의 메모리 주소는 " << ptr_int << "입니다." << endl;
	
	// cout << "double형 숫자의 값은 " << *ptr_double << "입니다." << endl;	
	// cout << "double형 숫자의 메모리 주소는 " << ptr_double << "입니다." << endl;
	
	// // delete ptr_int;
	// delete ptr_double;

	/////////////////////////////////////////////////////
	// 할당
    int* dynamicInt = new int;
    *dynamicInt = 42;

    std::cout << "Value of dynamicInt: " << *dynamicInt << std::endl;
	std::cout << "Address of ptr: " << dynamicInt << std::endl;

	// 해제
    delete dynamicInt;

    // 포인터를 삭제한 이후에는 해당 포인터를 사용하면 미정의 동작을 일으킬 수 있음
    // 삭제된 포인터를 사용하려고 하면 예상치 못한 결과가 발생할 수 있으므로 주의해야 함
    std::cout << "Value of dynamicInt: " << *dynamicInt << std::endl;
	std::cout << "Address of ptr: " << dynamicInt << std::endl;

	return 0;
}

