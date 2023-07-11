https://docs.python.org/ko/3.8/howto/argparse.html

https://docs.python.org/ko/3.8/library/argparse.html#the-parse-args-method
># arg parse *(인수 파싱)*
>`CLI` 파싱 모듈
>
>### 파서 생성, 인수 추가, 인수 파싱
>### 터미널
```
import argparse    # CLI 파싱 모듈
```
---

## 파서 생성
```angular2html
parser = argparse.ArgumentParser(
    prog='파서_이름',           # PROGram
    description='파서_설명',    # 인수 도움말 위
    epilog='파서_설명'          # 인수 도움말 아래
    )
```

## 인수 추가

+ ### 위치 인수 *(positional arguments)*
    ```angular2html
    parser.add_argument(
        '위치_인수',                
        action=None,          # 속성 초기값("stor_true": True, "store_false": False)
        default=None,         # 인수 초기값
        # type=None,          # 인수 자료형(None: 문자열, int: 정수)
        # choices=None,       # 인수 범위([자료형1, ..., 자료형n])
        help='설명',
        )
    ```

+ ### 옵션 인수 *(optional arguments)*
    `--인수`, `-알파벳 --인수`
    >중복O
    >```angular2html
    >parser.add_argument(
    >    '--인수',              
    >    # '-알파벳', '--인수',     
    >    action=None,          # 속성 초기값("stor_true": True, "store_false": False)
    >    default=None,         # 인수 초기값
    >    # type=None,          # 인수 자료형(None: 문자열, int: 정수)
    >    # choices=None,       # 인수 범위([자료형1, ..., 자료형n])
    >    help='설명',
    >    )
    >```
    >
    >중복X *(≒조건문)*
    >```
    >group = parser.add_mutually_exclusive_group()
    >
    >group.add_argument()
    >group.add_argument()
    >      ...
    >```

## 인수 파싱 
`args = parser.parse_args()`

+ ### 속성
  `args.인수`

---

## 터미널

+ ### 실행
  `python 파일.py 인수`: 위치

  `python 파일.py -알파벳 --인수`: 옵션
  >`python 파일.py -h`: 도움말 ★
  >```angular2html
  ># 형식(순서X)
  >usage: prog 파일.py [-h] 옵션_인수 위치_인수1 ... 위치_인수n
  >
  >description
  >
  >positional arguments:    # 위치 인수
  >  인수              설명
  >
  >optional arguments:    # 옵션 인수
  >  -h, --help     show this help message and exit
  >  -알파벳, --인수     설명
  >
  >epilog
  >```