>## Numpy
>파이썬 `배열` 라이브러리
> ### 정보
> ### 1차원, 2차원
---

## 정보
```
v = np.array( [0, 1, 2, 3] )
w = np.array( [ [1,2,3], [4,5,6] ] )    # 2차원
```

+ ### 크기 
  `변수.shape`
  ```
  v.shape    # (4, )
  ```

+ ### 차원 
  `변수.ndim`
  ```
  v.ndim    # 1
  ```

+ ### 최대값, 최소값
  `변수.argmax(axis = 축)`, `변수.argmin(axis = 축)`
  ```
  v.argmax()    # 3
  
  w.argmax()    # 5
  ```
  
---

## 1차원 *(Vector)*
axis = 0

+ ### 생성
  `np.array(리스트)`
  
    >0 ~ n-1: `np.arange(n)`
    >```
    >np.arange(5)    # [0, 1, 2, 3, 4]
    >```
    >i ~ j-1 ＆ k증가: `np.arange(i, j, k)`
    >```
    >np.arange(1, 10, 2)    # [1, 3, 5, 7, 9] 
    >```

---

## 2차원 (m×n Matrix)
axis = 1

+ ### 생성
    `np.array( [리스트1, ..., 리스트n] )`
  
    >영행렬: `np.zeros( (m,n) )`
    >
    >단위대각행렬: `np.eye(n)`
    >
    >모든 값(1): `np.ones( (m,n) )`
    >
    >모든 값(a): `np.full( (m,n), a )`
    >
    >임의의 값: `np.random.random( (m,n) )`

+ ### 변경
  >m행 ＆ 열 *(자동)*: `배열_변수.reshape(m,-1)`
  >
  >n열 ＆ 행 *(자동)*: `배열_변수.reshape(-1,n)`

---

## 소수점
```
np.round( 
    배열_변수
    , decimals = n      # 소수점 n+1번째에서 반올림 
    )
```
