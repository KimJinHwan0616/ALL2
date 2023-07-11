## 변수 ##
True = 12         # 예약어, 공백, 특수문자 사용 불가능
name = '김진환'    # 첫번째 글자는 무조건 영어
age = 99          # 두번째 글자부터 숫자, _ 사용 가능
Name = 123        # 변수명은 대·소문자 구분
###################################################################
## 자료형(Data Type): 숫자, 문자열, 리스트, 튜플, 사전, 집합, 불(True, False)

# 확인 #
# type(자료형)
type(True)  # bool      논리
type(age)   # int       정수
type(1.23)  # float     실수
type(name)  # str       문자

# 변환 # ★★
# 자료형(변환시킬 변수)
int(1.23)       # 실수->정수
str(Name)       # 정수->문자
float('123')    # 문자->실수
tuple([1,2,3])  # 리스트->튜플

###################################################################
## 연산자 ##

# 산술
a=10
b=5
print('a ** b=', a**b)    # 거듭제곱
print('a % b=', a%b)    # 나머지
print('a // b=', a//b)    # 몫

# 증가&감소
c=1
c += 1  # c = c + 1
c -= 1  # c = c - 1
c *= 5  # c = c * 5
c /= 3  # c = c / 3

# 비교 (==, !=, >, <, >=, <=)
print(a == b)   # a는 b다
print(a != b)   # a는 b가 아니다

# 논리 (and, or, not)
print(False and True)   # False
print(False or True)    # True
print(not a == b)   # a는 b가 아니다

# 멤버쉽 ( in, not in )
d = [1, 2, 3, 4, 5]
3 in d          # True
99 not in d     # True

###################################################################
# 입력 # : 입력값은 모두 문자열로 인식!!
# input(입력값)
input('숫자를 입력해보세요')

# 출력 #
# print(출력값1, ... ,출력값n, sep=' ', end=' ')
print(1.23, '문자', True, 99+21) # ,로 출력값 구분
print(12, 34, 56, sep='-')      # 값과 값을 -로 구분
print(12, 34, 56, end='*')      # 마지막 값에 *추가

###################################################################
## 문자열 ##
str1 = 'Hello'             # 추천
str2 = "I don't care"      # 작은따옴표를 포함시킬 경우
str3 = '''
Life is too short ...      # 여러줄일 경우
You need python        
'''
str4 = '''\
Life is too short ...      # 여러줄일 경우
You need python\        
'''

# \n    줄바꿈
# \t     Tab
# \\    \ 문자
# \'    ' 문자
# \"    " 문자
# \r    왼쪽공백(크롤링)

# 연결, 반복
'Hello,' + 'World!!'
'Hello' * 3

# 길이
len('Hello, World')

# 인덱스(번호)
str1[2]     # l
str1[-1]    # o

# 추출
str4 = 'Hello, World!!'

str4[:]        # 전체
str4[::2]      # 2의 배수 ★
str4[:5]       # Hello
str4[5:]       # , World!!
str4[-7:-2]    # World

# 변수 삽입
# f' {변수} '
name = '김진환'
age = 28
gender = '남'
f'이름: {name}, 나이: {age}, 성별: {gender}'

# f'{숫자: nd/.mf}'    # n자리수 고정 / 소수점 m+1째자리에서 반올림
f'{6: 2d}'            # 2자리수 고정
f'{3.1465: .2f}'      # 소수점 3째자리에서 반올림

# 수정
a = 'Python is the best'
# 문자열.lower()                   # 소문자
# 문자열.upper()                   # 대문자
# 문자열.count('문자')              # 문자 개수
# 문자열.find/index('문자')         # 문자 위치
a.find('y')         # 1
a.find('z')         # -1 : 찾는 문자열 없음
a.index('y')        # 1

# 문자열.lstrip/rstrip/strip()     # 왼쪽/오른쪽/양쪽 공백 지우기
# 문자열.replace('변경전 문자', '변경후 문자')

# 문자열.split( [구분기호] )         # 나누기 (리스트)
e = "Life is too short"
f = "a:b:c:d"
e.split()       # ['Life', 'is', 'too', 'short']
f.split(':')    # ['a', 'b', 'c', 'd']
