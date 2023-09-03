># IP 주소 *(Internet Protocol Adress)*
>`기기` 고유 번호 *(네트워크)*
>
>### 공인, 사설
>### URL
###### <img src = 'img/IP 주소.png'>
```
ipconfig    // Window
ifconfig    // Unix, Linux
```
---

## 공인 IP 주소
`고유` IP 주소
```
예) 개인, 회사
```

## 사설 IP 주소 *(로컬 IP, 가상 IP)*
`공유기` IP 주소
```
10.0.0.0 ~ 10.255.255.255
172.16.0.0 ~ 172.31.255.255
192.168.0.0 ~ 192.168.255.255

예) 집, 회사
```

+ ### 공유기
  ```
  하나의 '공인 IP 주소'로 
  '여러 대'의 컴퓨터를 인터넷에 연결할 수 있도록 하는 네트워크 기기
  ```

---

## URL *(Uniform Resource Locator)*
데이터 `IP 주소`
```angular2html
통신 규칙://호스트명.도메인//쿼리

예) https://
```

+ ### 도메인 *(domain)*
  `IP 주소` → `이름`
  ```angular2html
  예) naver.com, daum.net, ...
  ```
  >DNS *(Domain Name Server)*
  >```
  >웹 브라우저 → 요청 → 로컬 DNS → 응답 → 웹 브라우저
  >
  >예) 로컬 DNS, 루트 네임 서버, ...
  >```
  >>로컬 DNS: 클라이언트가 '가장 먼저' 접근하는 DNS

+ ### 웹 페이지 *(애플리케이션)*
    `클라이언트` ↔ `서버`
    ```angular2html
    메소드(클라이언트 → 서버)
    ▶ GET: 데이터 요청
    ▶ POST: 데이터 저장
    ▶ PUT: 전체 데이터 수정
    예) 게시글 수정
    ▶ PATCH: 일부 데이터 수정
    예) 게시글 조회수 증가
    ▶ DELETE: 데이터 삭제
    ▶ OPTIONS: URL 메소드 허용 목록
    
    메소드(서버 → 클라이언트)
    ▶ 1XX: 요청 O, 작업중
    ▶ 2XX: 요청 O, 전송 X
    ▶ 3XX: 전송 중
    ▶ 4XX: 클라이언트 오류
      401(Unauthorized): 로그인 X
      403(Forbidden): 로그인 O, 데이터 요청 권한 X 
      404(Not Found): 요청 데이터 없음 또는 URL 오류
    ▶ 5XX: 서버 오류
      502: 서버 과부하 또는 네트워크 문제
    ```


