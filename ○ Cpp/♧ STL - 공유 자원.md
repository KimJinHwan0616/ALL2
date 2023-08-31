## 뮤텍스
`#include <mutex>`
```angular2html
std::lock_guard<std::mutex> 객체(뮤텍스);    // 자물쇠 잠금, 해제
```

## atomic 
`<atomic>`
```
std::memory_order_relaxed  // 스레드 재배치 

// 변경 //
// (객체 == 값1) → (객체 = 값2)
이름.compare_exchange_strong(값1, 값2, 속성, 속성)    // 성공:true, 실패:false


// 동기화 //
std::memory_order_acquire  // 특정 작업 이전으로 스레드 재배치X
std::memory_order_release  // 특정 작업 이후로 스레드 재배치X
```
>선언: `atomic<자료형> 이름;`
>```
>std::atomic<bool> init;
>``` 
>초기화: `atomic<자료형> 이름(값);`
>```
>std::atomic<int> balance(100);
>``` 
>---
> 
>읽기: `이름.load(std::memory_order_acquire);`
>```
>balance.load(std::memory_order_acquire);
>```
>쓰기: `이름.store(값, std::memory_order_release);`
>```
>balance.store(50, std::memory_order_release); 
>``` 


