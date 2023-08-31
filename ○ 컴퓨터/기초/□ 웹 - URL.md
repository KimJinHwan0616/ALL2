># URL *(Uniform Resource Locator)*
>`자료` *(이미지, PDF, ...)* 의 `위치`를 나타내는 `주소` 
>### IP 주소
>### 도메인, 호스트명, 통신 규칙
```angular2html
URL = 통신 규칙://호스트명.도메인//쿼리
```

---

## IP 주소 *(Internet Protocol Adress)*
기기와 연결된 `네트워크 끝 단`의 주소 *(≒네트워크 고유 번호)*
```angular2html
IP: 데이터 통신 규칙

IPv4: 숫자(0~255) - 4마디
예) 0.0.0.0 ~ 255.255.255.255

IPv6: 16진수(0~F) - 4자리 - 8마디
예) 1234:5678:9ABC:DEF0:1234:5678:9ABC:DEF0
```

+ ### 공인 IP 주소 
  서로 `중복되지 않는` 고유 IP 주소
  ```
  예) 개인, 회사
  ```
 
+ ### 사설 IP 주소 *(로컬 IP, 가상 IP)*
  `공유기`의 고유 IP 주소
  ```
  공유기: 하나의 '공인 IP 주소'로 '여러 대'의 컴퓨터를 인터넷에 연결할 수 있도록 하는 네트워크 기기
  
  예) 집, 회사
  ```

+ ### 고정 IP 주소
  고정적으로 `변경되지 않는` IP 주소
  ```
  예) 기업＆기관
  ```

+ ### 유동 IP 주소
  유동적으로 `변경되는` IP 주소
  ```
  예) 가정
  ```

---

## 도메인 *(domain)*
`IP 주소`를 `이름`으로 `변환`한 것
```angular2html
예) naver.com, daum.net, ...
```

+ ### DNS *(Domain Name Server)*
  `도메인`을 `IP 주소`로 `변환`하는 `서버`
  ```
  브라우저 → 도메인 요청 → 로컬 DNS → IP 주소 전송 → 브라우저
  
  예) 로컬 DNS, 루트 네임 서버, ...
  ```
    
  >로컬 DNS 
  >```
  >클라이언트가 '가장 먼저' 접근하는 DNS
  >```

## 호스트명
```angular2html
예) www, mail, search, ...
```

+ ### www *(W3, World Wide Web)*
  `전 세계 사람`이 정보를 공유하고 소통할 수 있는 `인터넷 공간`

  >하이퍼텍스트 *(hypertext)*
  >```
  >'링크'를 '클릭'해서 '웹 사이트'를 접속하는 텍스트
  >
  >예) 뉴스, 블로그, ...
  >```

## 통신 규칙 *(protocol)*
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

+ ### HTTP *(HypterText Transfer Protocol)*
  `요청(클라이언트)`＆`응답(서버)`


+ ### HTTPS *(HypterText Transfer Protocol Secure socket layer)*
  `보안 기능` + `HTTP`

