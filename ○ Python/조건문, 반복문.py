## 조건문 ##
# if 조건식1:
#    실행문
# elif 조건식2:
#    실행문
#     ...
# else:
#    실행문

# 국어, 영어, 수학 점수를 입력받아 총점, 평균을 계산하고
# 평균은 '수우미양가' 조건으로 출력
kor = int(input('국어: '))
eng = int(input('영어: '))
mat = int(input('수학: '))

tot = kor + eng + mat
avg = tot / 3
grd = ''

if avg >= 90:
    grd = '수'
elif avg >= 80:
    grd = '우'
elif avg >= 70:
    grd = '미'
elif avg >= 60:
    grd = '양'
else:
    grd = '가'

print(kor, eng, mat, tot, avg, grd)

###################################################################
## 반복문(for, while) ##
# 반복 횟수가 명확한 경우 : for
# 반복 횟수가 명확하지 않은 경우 : while
# 중지 : break / 제외 : continue
# 무한 반복 : while True #

# 반복 횟수가 명확한 경우 #
# for 변수 in range(시작값, 종료값+1, [증가값]) / 리스트 / 튜플:
#       실행문

# 1 ~ 100 정수 출력
for x in range(1, 101):
    print(x, end=' ')       # end : 줄바꿈 -> 공백

# 100 ~ 1 정수 출력
for x in range(100, 0, -1):
    print(x, end=' ')

# 1 ~ 100 정수 중 짝수 출력
for x in range(1, 101, 2):
    print(x, end=' ')

# 1 ~ 100 정수의 합
sum = 0
for x in range(1, 101):
    sum += x

print(sum)

# 1 ~ 100 정수 중 3의 배수이고 2의 배수가 아닌 정수, 누적합 출력
sum = 0

for x in range(1, 101):
    if (x % 3) == 0 and (x % 2) != 0:
        sum += x
        print(x, end=' ')

print(sum)

# while: 반복 횟수가 명확하지 않은 경우 #
# 변수 = 초기값
#
# while 조건식:
#     실행문
#     증감식

# 1 ~ 100 정수 출력
x = 1
while x <= 100:
    print(x, end=' ')
    x += 1

# 1 ~ 100 정수 중 홀수 출력
x = 1
while x <= 100:
    print(x, end=' ')
    x += 2

# 3의 배수 이고 2의 배수가 아닌
# 1 ~ 100 모든 정수의 합 출력
x = 1
sum = 0

while x <= 100:
    if (x % 3) == 0 and (x % 2) != 0:
        sum += x
    print(x, end=' ')
    x += 1

print(sum)

# break : 중지 #
# 1 ~ 100 모든 정수의 합 출력
# 단, 누적합이 1500을 넘으면 반복문 중지
x = 1
sum = 0

while x <= 100:
    if sum >= 1500: break   # 조건 만족시 반복문 종료
    sum += x
    x += 1

# 제외 : continue #
# 1 ~ 100 모든 정수의 합 출력
# 단, 7의 배수 또는 9의 배수 제외
x = 0
sum = 0

while x <= 100:
    x += 1
    if x % 7 == 0 or x % 9 == 0: continue
    sum += x

print(x, sum)

# 무한 반복 #
# while True:
#       실행문

# 로그인
while True:
    id = input('아이디: ')
    passwo = input('비밀번호: ')

    if id == 'abc123' and passwo == '987xyz':
        print('로그인 성공')
        break  # 로그인 성공시 반복 실행 중지
    else:
        print('로그인 실패')

