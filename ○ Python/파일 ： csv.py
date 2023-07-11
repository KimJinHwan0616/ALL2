## 텍스트(csv) 파일 ## : 각 행의 컬럼값이 쉼표로 구분된 텍스트 형식 파일(comma separated values)
# 철수, 56, "34,500원"       # 컬럼값에 ,가 포함된 값일 경우 " "로 감싸기
# 호출, 열기(생성)&쓰기, 열기&읽기
# 파일_모드
# 'w' : (기존 데이터를 초기화하고)쓰기
# 'a' : (기존 데이터에 추가해서)쓰기
# utf-8 : 한글 인코딩_형식
# newline : 입력 후 줄바꿈X

# 호출
# import csv      # 외부 모듈 csv 호출

# 열기(생성)&쓰기
# with open( '[상위폴더/]파일_이름.형식', 파일_모드, newline=''[, encoding='인코딩_형식'] ) as 파일_별칭:
#   변수 = csv.writer/DictWriter(파일_별칭)
#   쓰기 형태

import csv
score = [ ['헤교',54,50,44], ['지현',43,95,87], ['수지',77,86,20] ]

with open('score.csv', 'w', newline='', encoding='utf8') as c:
    a = csv.writer(c)
    a.writerows(score)       # 모두 쓰기

with open('score.csv', 'w', newline='', encoding='utf-8') as c:
    w = csv.writer(c)
    for row in score:         # 행 단위로 한 줄씩 쓰기
        w.writerow(row)

# 열기&읽기
# with open( '[상위폴더/]파일_이름.형식', [encoding='인코딩_형식'] ) as 파일_별칭:
#   변수 = csv.reader/DictReader(파일_별칭)
#   [next 변수]               # 첫 행 제외
#   읽기 형태

import csv

with open('score.csv', encoding='utf8') as c:
    a = csv.reader(c)
    print(list(a))           # 리스트로 변환해서 모두 읽기

with open('score.csv', encoding='utf8') as c:
    a = csv.DictReader(c)
    for i in a:             # 각 행을 사전으로 변환해서 읽기
        print(i)

with open('score.csv', encoding='utf8') as c:
    a = csv.reader(c)
    next(a)                   # 첫번째 행(ex 컬럼) 제외 
    for row in a:             # 각 행을 리스트로 변환해서 읽기
        print(row)

