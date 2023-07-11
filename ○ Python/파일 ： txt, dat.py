## 텍스트(txt, dat) 파일 ##
# 열기(생성)&쓰기, 열기&읽기
# 파일_모드
# 'w' : (기존 데이터를 초기화하고)쓰기
# 'a' : (기존 데이터에 추가해서)쓰기
# utf-8 : 한글 인코딩_형식
# read : 모두 읽기 / readline : 한줄씩 읽기 (반복문) / readlines : 한줄씩 모두 읽기


# 열기(생성)&쓰기 #
# with open( '[상위폴더/]파일_이름.형식', 파일_모드, [encoding='인코딩_형식'] ) as 파일_별칭:    # 열기(생성)
#       파일_별칭.write(코드)                                                             # 쓰기

# 이름,국어,영어,수학 점수를 입력받은 값을 score.dat 파일에 저장
name = input('이름: ')
kor = int(input('국어: '))
eng = int(input('영어: '))
mat = int(input('수학: '))

info = f'{name}, {kor}, {eng}, {mat}\n'

with open('score.dat', 'a', encoding='utf-8') as d:
    d.write(info)

# 열기&읽기 #
# with open( '[상위폴더/]파일_이름.형식', [encoding='인코딩_형식'] ) as 파일_별칭:    # 열기
#       변수 = 파일_별칭.read/readline/readlines()                                     # 읽기

# read
with open('score.dat', encoding='utf-8') as d:
    data = d.read()

print(data)

# readline #
# score.dat 파일을 읽어서 "이름/국어/영어/수학"으로 표시
with open('score.dat', encoding='utf-8') as d:
    while True:
        row = d.readline()       # 한 줄 씩 읽기
        if not row: break        # 읽을 내용이 없다면 읽기 중지
        li = row.split(', ')     # 이름, 국어, 영어, 수학을 ,로 구분해서 리스트에 저장
        info = f'{li[0]}/{li[1]}/{li[2]}/{li[3]}'
        print(info, end='')

# readlines #
with open('score.dat', encoding='utf-8') as d:
    rows = d.readlines()        # 한 줄 씩 구분하여 리스트에 저장
    for row in rows:
        print(row)
