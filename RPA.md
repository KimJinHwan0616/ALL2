https://drive.google.com/file/d/18Yk3n4Ldl08TiR7KM1V8UdsS0gsu6xsi/view?ts=633b8329
https://drive.google.com/file/d/1uLmtH1BrbwDTiqr5NoDSrSoXD8aPT5UU/view?ts=6344bc7e

># RPA
>로봇 프로세스 자동화(Robotic Process Automation)
>
>### 단축키, 액티비티, 변수, 자료형, 에러, ...

---
## 단축키
+ ``F2`` : 이름 변경　　　　``Shift+F2`` : 주석　　　　``Shift+F5`` : 실행

---
## 액티비티
개발도구

+ ### 메인
  >``Sequence`` : 순서<br>
  >``Flowchart`` : 순서도<br>
  >``Assign`` : 변수 설정<br>
  >~~``input Dialog`` : 입력창~~<br>
  >``Write Line`` : 출력창

+ ### 조건문
  >``IF`` : Sequence<br>
  >``Flow Decision`` : Flowchart

+ ### 반복문
  > ``While, Do while`` : 일반<br>
  > ``For Each`` : 배열, 리스트<br>

+ ### 엑셀
  >``Excel Apllication Scope`` : 불러오기(주소)<br>
  >　　　　``Write Range`` : 쓰기<br>
  >　　　　``Read Range`` : 읽기
  > 
  >~~``Append Range`` : 데이터 테이블 추가~~

+ ### 데이터 테이블
  >``Build Data Table`` : 생성<br>
  >``Filter Data Table`` : 조건 검색<br>
  > 
  >``For Each Row in Datatable`` : 반복문<br>
  >　　　　``Add Data Row`` : 행 추가(한줄)<br>

+ ### 브라우저
  >``Open Browser`` : 열기<br>
  >　　　　``Maximize Window`` : 확대<br>
  > 
  >``Attach Browser`` : 속성 인식<br>
  >　　　　``Navigate To`` : 사이트 이동<br>
  >　　　　``Close Tab`` : 닫기(1개)
  > 
  >``Delay`` : 지연
  >
  >``Click`` : 마우스　　　　``Type into`` : 키보드<br>
  >``Get Password`` : 패스워드 암호화<br>
  >``Send HotKey`` : 단축키 
  >
  >``Get Text`` : 텍스트 추출(문자열)<br>
  >``Select Item`` : 드롭박스 목록 선택

+ ### 예외처리
  >``Element Exists`` : If / Flow Decision<br>

---
## 변수
+ ### 선언
  >``String`` : 문자열(Str)　　　　``Int32`` : 정수(Int)　　　　``Double`` : 실수(Dbl)<br>
  > 
  > ``Array of [T]`` : 배열(String[], Int32[], ...)　　　　``List`` : 리스트　　　　``Dictionary`` : 사전<br>
  > 
  >``Datatable`` : 표(DT)<br>
  > 
  >``Boolean`` : 참, 거짓(Bln)<br>
  > 
  >``GenericValue`` : 모든 타입 자료형

+ ### 초기화
    ```
    String = ""　　　　
    Int32 = 0
    Array = new [type](크기-1){}
    List = new List(of Type)
    Datatable = new Datatable()
    Object = Nothing
    Dictionary = new Dictionary(of String, Object)
    ```
---
## 자료형
+ ### 변환
  ``자료형.Tostring`` : 문자열<br>
  ``cdbl(자료형)`` : 실수　　　　``cint(자료형)`` : 정수

---
## 에러
+ ### 셀렉터
  이 선택기에 해당하는 UI엘리먼트를 찾을 수 없습니다
  ```angular2html
  빨강색 - 화면, 로직 확인
  
  초록색 - UI 한 번 더 클릭
  ```
+ ### 변수
  개체참조의 인스턴스로 설정되지 않았습니다
  ```angular2html
  속성 출력 빈 칸 확인(Read Range, Bulid Data Table, ...)
  
  변수 확인(For Each Row in Datatable, Add Data Row, ...)
  ```
  

---
