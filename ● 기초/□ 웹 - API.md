># API *(Application Programming Interface)*
>`소프트웨어 ↔ 소프트웨어` 인터페이스
> 
>### AJAX
---


## AJAX *(Asynchronous Javascript And Xml)*
`데이터`를 `자바스크립트`로 `서버`에 `비동기 방식`으로 `요청`하는 것
```angular2html
동기 방식: 서버로 요청했을 때, 응답이 돌아와야 다음 동작 실행

비동기 방식: 서버로 요청했을 때, 응답 상태와 상관없이 다음 동작 실행
예) 검색어 자동완성, 장바구니 아이템 추가, ...
```

+ ### DOM *(Document Object Model)*
    `자바스크립트`로 `HTML 요소`를 제어할 수 있는 `객체`
    ```
    예) body, head, div, img, ...
    
                 Document
                    │
                   html
        ┌───────────┴───────────┐
      head                     body
                                │
                       ┌────────┼────────┐
                       h1      input    button  
    ```
