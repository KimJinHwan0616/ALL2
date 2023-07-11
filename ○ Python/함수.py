## 함수(모듈, 사용자 정의, 내장) ##
# 입력값(과일) -> 함수(믹서기) -> 결과값(과일주스)

# 모듈_함수 #
# 호출
# import 모듈_함수 [as 별칭]
import random       # random 모듈
import random as r  # 별칭이 r인 random 모듈

# 실행
# 모듈_함수.기능(변수)
# 별칭.기능(변수)
r.seed(3)                  # 초기값 지정
random.random()            # 0~1 임의의 실수 생성
r.randint(2, 6)            # 2~6 임의의 실수 생성

# 1 ~ 45 임의의 정수 6개 생성 (중복O)
import random 

for _ in range(6):  # 변수 X
    print(random.randint(1, 45), end=' ')

###################################################################
# 사용자 정의 함수 # : 일반, 입력값X , 결과값X, 입력값&결과값X

# 일반 #
# def 함수_이름( 매개변수1, ... 매개변수n[, *args] ):
#    실행문
#    return 결과값
# 
# 함수_이름( 인수 )        # 실행
def add(a, b):
    result = a + b
    return result

add(3, 4)

# 입력값X #
def say():
    return 'Hi'

say()

# 결과값X #
def add(a, b):
    print(f'{a} + {b} = {a+b} 입니다')

add(1, 2)

# 입력값&결과값X #
# 1 ~ 100 정수 출력 함수
def int_number():
    for i in range(1, 100):
        print(i, end=' ')

int_number()

# 전역 변수
# global 변수
def f():
    global s    #
    s = "런던"
    return s
s = "뉴욕"

print(s)     # 런던O, 뉴욕X

# 가변 인수
def cal_add(a, *args):
    print(f'{a}+ {args}={a + sum(args)}')

cal_add(1, 2, 3, 4)
###################################################################
# 내장 함수 #

# zip : 자료구조 일대일대응(=순서쌍)
# zip( 자료구조1 , 자료구조2, ...)      # 자료구조1 & 자료구조2 일대일대응
a = zip({1, 3, 5}, {2, 4, 6})
b = zip([2, 3], [1, '국어', True])

# dict(a)     # 확인용
# tuple(a)    # 확인용
# list(b)     # 확인용
# set(a)      # 확인용

# enumerate : 순서자료구조 인덱스 추가 출력
# for x, y in enumerate( 순서자료구조 ): 순서자료구조 인덱스 번호 추가 출력
#   print(x, y)

for x, y in enumerate( ['book', 'back', 'shoes'] ):
    print(x, y)



