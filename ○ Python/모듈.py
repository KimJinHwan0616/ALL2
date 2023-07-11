## 모듈 ## : 함수/클래스 모아 놓은 파이썬 파일
# 호출, 실행
# 사용자 정의 모듈
# 내부 모듈 : builtins, 호출X
# 외부 모듈 : 터미널 -> Command Prompt -> pip install 모듈

# 호출
# import 모듈 [as 별칭]
import builtins     # 내부 모듈

import random       # random 모듈
import random as r  # 별칭이 r인 random 모듈

# 실행
# 모듈.기능(변수)   # 실행
# 별칭.기능(변수)             # 별칭으로 실행

r.seed(3)                  # 초기값 지정
random.random()            # 0~1 임의의 실수 생성
r.randint(2, 6)            # 2~6 임의의 실수 생성

# 조회 (기능)
# dir(모듈)

dir(builtins)     # 내부 모듈 기능
dir(random)       # 모듈 random 기능
dir(r)            # 별칭으로 모듈 random 기능

