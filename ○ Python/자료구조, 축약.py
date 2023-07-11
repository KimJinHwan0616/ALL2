## 자료구조 ##
# 데이터 순서O : 리스트list, 튜플tuple
# 데이터 순서X : 집합set, 사전dict
# 데이터 변경 : 튜플,집합 -> 리스트 -> 튜플,집합
# 중복 제거 : 리스트,튜플 -> 집합 -> 리스트,튜플
# 축약 : 리스트,집합 / 사전

# 리스트 # : 데이터 순서O, 데이터 변경O
# 변수 = [값1, 값2, ..., 값n]

# 생성
menus = []      # 빈 리스트
menus = ['떡볶이', '순대']

# 추가
menus.append('라면')

# 연결, 반복
menus + ['튀김']
menus * 2

# 길이
len(menus)

# 인덱스(번호)
menus[0]    # 라면
menus[2]    # 짜장면

# 삽입
menus.insert(1, '냉면')   # 지정한 번호에 삽입

# 수정
menus[1] = '비빔면'    # 지정한 인덱스에서 수정

# 삭제
menus.remove('라면')  # 지정한 값으로 삭제
menus.pop(0)     # 지정한 번호로 삭제
menus.pop()     # 마지막 리스트 삭제

# 정렬
numbers = [12, 5, 30, 70]

numbers.sort()   # 오름차순
numbers.sort(reverse= True)  # 내림차순

# 지현, 헤교, 수지의 국어, 영어, 수학 점수에 대해
# 총점, 평균을 구하고 평균에 대해 '수우미양가'로 출력
# 지현: 99, 76, 56 / 혜교: 98, 56, 86 / 수지: 99, 66, 96

names = ['수지', '헤교', '지연']
kors = [99, 76, 56]
engs = [98, 56, 86]
mats = [99, 66, 96]

# tots = [●, ●, ●]
# tots[0] = kors[0] + engs[0] + mats[0]
# tots[1] = kors[1] + engs[1] + mats[1]
# tots[2] = kors[2] + engs[1] + mats[2]     # 성적처리

tots = []
avgs = []
grds = []

for i in range(3):
    tots.append(kors[i] + engs[i] + mats[i])
    avgs.append(tots[i] / 3)
    if avgs[i] >= 90: grds.append('수')
    elif avgs[i] >= 80: grds.append('우')
    elif avgs[i] >= 70: grds.append('미')
    elif avgs[i] >= 60: grds.append('양')
    else: grds.append('가')

    print(f'국어: {names[i]}, 영어: {engs[i]}, 수학: {mats[i]}\n'
          f'총점: {tots[i]}, 평균: {avgs[i]:.1f}, 학점: {grds[i]}')


###################################################################
# 튜플 # : 데이터 순서O, 데이터 변경X
# 변수 = (값1, 값2, ..., 값n)

# 생성
city = ()   # 빈 튜플
city = ('서울', '부산', '대전')

# 변경 ( 튜플 -> 리스트 -> 튜플 )
city = list(city)
city[1] = '대구'
city = tuple(city)

###################################################################
# 집합 # : 데이터 순서X, 데이터 변경O, 데이터 중복X
# 변수 = {값1, 값2, ..., 값n}

# 생성
# zero = {}       # dict = {}으로 인식하므로 불가능
zero = {1, 2}

# 추가
zero.add('수열')       # 하나
zero.update({3,4,5})  # 여러개

# 제거
zero.remove(4)  # 하나
zero.clear()    # 모두

# 변경 ( 집합 -> 리스트 -> 집합 )
zero = list(zero)
zero[1] = 88
zero = set(zero)

###################################################################
# 사전 # : 데이터 순서X, 데이터 변경O, 키 중복X, 값 중복O
# 변수 = {'키1':'값1', '키2':'값2', ..., '키n':'값n'}

# 생성
info = {}   # 빈 사전
info = {'name':'홍길동', 'age':35, 'gender':'남자'}

# 추가
info['pay'] = 350

# 검색
info.get('name')

# 변경
info['age'] = 25

# 삭제
del info['gender']

###################################################################
## 축약 ## : 리스트, 집합 / 사전
# 리스트, 집합 # : 반복문, 이중 반복문, 조건문, 다중 조건문

# 반복문
# 변수 = [ 값 for (범위)변수 in 범위 ]

# 1 ~ 10 정수를 리스트에 저장
a = [ i for i in range(1,11)]

# 0 ~ 20 짝수를 집합에 저장
b = { i for i in range(0,21,2) }

# 1 ~ 59 5의 배수 리스트에 저장
c = [5*i for i in range(1,60)]

# 이중 반복문
# 변수 = [ 식 for (범위)변수1 in 범위 for (범위)변수2 in 범위 ]

# 구구단 중 7,8단의 결과값만 출력해서 리스트에 저장
# [7,14,21,...,63, 8,16,24,...,72]
gugudan78 = [ i*j for i in range(7,9)
           for j in range(1,10) ]

# 조건문
# 변수 = [ 값 for (범위)변수 in 범위 if 조건 ]

# 리스트에서 정수만 뽑아 제곱값을 계산하여 새로운 집합에 생성
list1 = [1,2,'A',False,9,100]

list2 = {i**2 for i in list1 if type(i) == int}

# 다중 조건문
# 변수 = [ 조건값 if 조건 else 나머지값 for (범위)변수 in 범위 ]

# 1 ~ 100 임의의 정수가 짝수면 even,
# 홀수면 odd라고 구분해서 리스트에 저장
d = ['even' if i % 2 == 0 else 'odd'
     for i in range(1,101)]


# 사전 # : 반복문, 조건문

# 반복문
# { 키_변수 : 값_변수
#           for 키_변수, 값_변수 in zip(키_리스트, 값_리스트) }

# name, gender 리스트의 값들을 사전에 저장
name = ['헤교', '우빈', '수지']   # 키
gender = ['여자', '남자', '여자']   # 값

list3 = { x : y for x,y in zip(name, gender)}

# 조건문
# { 키_변수 : 조건값 if 조건 else 나머지값]
#           for 키_변수, 값_변수 in zip(키_리스트, 값_리스트) }

# 이름, 점수 리스트에서
# 학생은 키로, 합격여부는 값으로 사전에 저장
# 단, 85점 이상인 경우 '합격', 그 외는 '불합격'이라고 출력
name = ['철수', '영희', '길동', '꺽정']
score = [50, 80, 90, 60]

list4 = { x : '합격' if y>=85 else '불합격' for x,y in zip(name, score)}
