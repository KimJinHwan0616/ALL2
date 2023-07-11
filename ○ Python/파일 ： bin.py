## 이진(bin) 파일 ## : 자료형을 byte로 저장된 파일 ( ≒ pickle 파일 )
# 이미지파일, 동영상파일, ...
# 호출, 열기(생성)&쓰기, 열기&읽기
# 파일_모드
# 'wb' : (기존 데이터를 초기화하고)쓰기
# 'ab' : (기존 데이터에 추가해서)쓰기
# utf-8 : 한글 인코딩_형식
# pack : 자료형 -> byte 
# unpack : byte -> 자료형
# 서식지정문자(자료형: byte제한)
# c(문자: 1), b(자연수,0: 1), i(자연수,0: 4),
# f(소수점4번째자리까지실수: 4), d(소수점8번째자리까지실수: 8),
# s(문자열: 없음)

# 호출 #
# import struct     # 외부 모듈 struct 호출

# 열기(생성)&쓰기
# with open('[상위폴더/]파일_이름.bin', '파일_모드') as 파일_별칭:      # 열기
#     변수 = struct.pack('서식지정문자n', 변수1, ... ,변수n)          # 자료형 -> byte
#     파일_별칭.write(변수)                                         # 쓰기

# 읽기&출력 #
# with open('[상위폴더/]파일_이름.bin', '파일_모드') as 파일_별칭:      # 열기
#     파일_별칭.read(변수)                                          # 읽기

# 예1
import struct

kor = 99
eng = 98
mat = 99

with open('score.bin', 'wb') as b:
    score = struct.pack('iii', kor, eng, mat)   # 99,98,99(2byte) -> byte
    b.write(score)

with open('score.bin', 'rb') as b:
    b.read()
    #a = b.read()  # 확인용
#type(a)           # 확인용
#print(a)          # 확인용

# 예2
import struct

name = '혜교'
tot = 297
avg = 99.8

#with open('score1.bin', 'wb') as b:           # 문자열은변경시켜서 값을 넣어야함
     #info = struct.pack('sif', name, tot, avg) # 문자열은 서식지정문자에 없으므로 정수로 변경시켜서 값을 넣어야함
     #b.write(info)

with open('score1.bin', 'wb') as b:
    info = struct.pack('iiiiif', len(name), kor, eng, mat, tot, avg)  # 문자열(name) -> 정수
    b.write(info)

with open('score1.bin', 'rb') as b:
    b.read()
    #a = b.read()
#print(a)
