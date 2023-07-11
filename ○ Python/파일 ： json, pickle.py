## json 파일 ## : 자바스크립트 ★사전★ 형식 파일(javascript object notation)
# 호출, 열기(생성)&쓰기, 열기&읽기
# 파일_모드
# 'w' : (기존 데이터를 초기화하고)쓰기
# 'a' : (기존 데이터에 추가해서)쓰기
# ensure_ascii=False : ascii X , 한글 O

# 호출
# import json   # 외부 모듈 json 호출
# (사전)변수 = 사전

# 열기(생성)&쓰기
# with open('[상위폴더/]파일_이름.json', '파일_모드'[, encoding='인코딩_형식'] ) as 파일_별칭:
#   json.dumps( (사전)변수, 파일_별칭 [, ensure_ascii=False] )

# 열기&읽기
# with open('[하위폴더/]파일_이름.json', '파일_모드' ) as 파일_별칭:
#   json.loads(파일_별칭)

import json
score = {'name':'헤교', 'kor':99, 'eng':98,'mat':97}

with open('score.json', 'w', encoding='utf8') as j:
    json.dump(score, j, ensure_ascii=False)
    #j = json.dump(score, j, ensure_ascii=False)
#print(j, type(j))           # 확인용

with open('score.json', encoding='utf8') as j:
    json.load(j)
#    a = json.load(j)   # 확인용
#print(a)               # 확인용

###################################################################
## pickle 파일 ## : 자료구조를 byte로 저장된 파일 ( ≒ bin 파일 )
# 호출, 열기(생성)&쓰기, 열기&읽기
# 'wb' : (기존 데이터를 초기화하고)쓰기
# 'ab' : (기존 데이터에 추가해서)쓰기
# 'rb' : 읽기
# utf-8 : 한글 인코딩_형식

# 호출
# import pickle     # 외부 모듈 pickle 호출
# (자료구조)변수 = 자료구조

# 열기(생성)&쓰기
# with open('[상위폴더/]파일_이름.pkl', '파일_모드' ) as 파일_별칭:
#   pickle.dump( (자료구조)변수, 파일_별칭 )

# 열기&읽기
# with open('sungjuk.pkl', '파일_모드') as 파일_별칭
#   pickle.load(파일_별칭)

import pickle
score = {'name':'헤교', 'kor':99, 'eng':98,'mat':97}

with open('score.pkl', 'wb') as p:
    pickle.dump(score, p)
    #a = pickle.dump(score, p)
#print(a)   # 확인용
#type(a)    # 확인용

with open('score.pkl', 'rb') as p:
    pickle.load(p)
    #b = pickle.load(p)
#print(b)   # 확인용
#type(b)    # 확인용
