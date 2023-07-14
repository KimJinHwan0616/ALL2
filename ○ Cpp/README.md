http://www.tcpschool.com/cpp/intro
># C++
>`소스 코드` 파일 *(.cpp)* → 선행처리기 → 컴파일러 → `목적 코드` 파일 *(.o, .obj)* → 링킹 → `실행` 파일 *(.exe)*
> 
>### 선행처리기
###### <img src = 'img/C++.png'>
```
링커(linker): 오브젝트 파일 + 시동 코드(start-up code) + 표준 라이브러리 파일

혼공컴운 p.87
```
######
```angular2html
#include 모듈    // import 모듈
using namespace 네임스페이스;    // import 네임스페이스

반환_자료형 main()
{
    실행문;
}
```
---

## 선행처리기 *(preprocessor)*

+ ### 헤더 파일
  `#include`: load

+ ### 매크로
  `#define 매크로 상수_또는_문자열`
  ```
  #define SIZE 100
  
  char ch[size]        //char ch[100];
  for(i=0: i<SIZE; i++)    // for(i=0;i<100;i++)
  ```