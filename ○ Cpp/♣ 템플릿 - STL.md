># STL *(Standard Template Library)*
>표준 `템플릿` 라이브러리 ★
>
>### 반복자, 반복문
```
#include <algorithm>

// 이동(A → B) //
std::move(
    A객체_이름.begin(), A객체_이름.end(), 
    B반복자);
    
B객체_이름 = std::move(A객체_이름);

// 추가 //
객체_이름.emplace(값, 값)  
``` 
---

## 반복자
```
remove_if( 이름.begin(), 이름.end(), 람다함수);    // 삭제 
```
>시작: `객체_이름.begin();`
> 
>끝: `객체_이름.end();`
 
## 반복문
```
for_each(
    객체_이름.begin(), 객체_이름.end(), 
    함수);
```
```
void PrintSquare(int num) {
    std::cout << num * num << " ";
}

std::vector<int> numbers = {1, 2, 3, 4, 5};

// 각 숫자의 제곱값 출력
std::for_each(
    numbers.begin(), numbers.end(), 
    PrintSquare);
```


