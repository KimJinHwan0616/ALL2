https://neo4j.com/docs/getting-started/5/cypher-intro/
># 패턴
>`노드`, `엣지`
>
>### 노드, 엣지
>### 생성, 선택, 반환, 수정, 삭제
```angular2html
MATCH 패턴, ..., 노드
WHERE 조건
WITH 객체 as 변수1, ..., 객체 as 변수n  // 변수 선언+초기화 

OPTIONAL MATCH
UNWIND

RETURN 
ORDER BY [DESC]
LIMIT 숫자
```
---

## 노드 *(Node labels)*
>선택: `(노드_변수:라벨)` ★
>
>생성: `(:라벨)`
>
>전체: `(n)`
```angular2html
예) (p:Person), (:Movie), (n)
```

+ ### 속성
  `(노드_변수:라벨 {속성1: 값, ..., 속성n: 값})`
  ```
  예) (p:Person {name: "Sally", data: [0,1,2]})
  ```

## 엣지 *(Relationship types)*
>선택: `노드 -[엣지_변수:유형]-> 노드` ★
>
>생성: `노드 -[:유형]-> 노드` 
>
>전체: `()-[r]->()`
```
예) (p) -[r:LIKES]-> (m:Movie)
```

+ ### 속성
  `노드 -[엣지_변수:유형 {속성1: 값, ..., 속성n: 값}]-> 노드`
  
+ ### 노드
  >시작: `startNode(엣지_변수)`
  >
  >끝: `endNode(엣지_변수)`
  
---

## 생성
>`MERGE 패턴1, ..., 패턴n`: 중복X
> 
>`CREATE 패턴1, ..., 패턴n`: 중복O

## 선택 ★
`MATCH 패턴1, ... , 패턴n`

## 요소

+ ### 속성
  >일반: `변수.속성`
  > 
  >id, type: `속성(변수)`

+ ### 통계
  >개수: `count(변수)`
  >
  >크기 *(리스트)*: `size(변수.리스트_속성)`

## 변수 선언+초기화
`WITH 요소1 as 변수1, ..., 요소n as 변수n`

## 반환
`return 요소1, ..., 요소n`

## 수정＆추가
`SET 변수.속성 = 값`

## 삭제
>노드(엣지X), 엣지: `DELETE 변수`
>
>노드(엣지O): `DETATCH DELETE 변수`
>
>속성: `REMOVE 변수.속성`
```
전체
match (n)
detach delete n

엣지
MATCH ()-[r]->()
DELETE r
```




