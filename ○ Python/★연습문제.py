## 조건문, 반복문 ##
# 1 ~ 10 숫자 맞추기 프로그램
magic = random.randint(1, 10)

while True:
    key = int(input('숫자를 하나 입력하세요 : '))

    if magic == key:
        print('성공!')
        break
    elif magic < key:
        print('숫자가 너무 커요!')
    else:
        print('숫자가 너무 작아요')

# 3자리 복권 당첨 프로그램
key = input('3자리 복권 숫자: ')
magic = str(random.randint(100, 999))
match = 0  # 일치여부

for x in range(3):
    if magic[x] == key[0] or magic[x] == key[1] \
            or magic[x] == key[2]:
        match += 1      # 복권 자리 수 비교

if match == 3:
    print('당첨! 상금 100만원 지급!')
else:
    print('아쉽지만 다음기회를!', key, magic, match)     # 복권 당첨 판단

# 1 ~ 9 숫자 하나를 입력 받아서 구구단을 출력
# 단, 1 - 9 이외의 숫자 또는 문자를 입력 받으면 “잘못 입력하셨습니다!!” 출력
dan = input("단을 입력하시오: ")

if dan in ('1', '2', '3', '4', '5', '6', '7', '8', '9'):
    for x in range(1, 10):
        print(f'{dan}*{x}={int(dan) * x}')
else:
    print('잘못입력하셨습니다!!')

# 소문자를 입력 받아 대문자로 출력
# 단, 소문자 이외의 숫자나 문자를 입력 받으면 “잘못 입력하셨습니다!!” 출력
# ASCII 코드(파이썬) : ord(문자or숫자) / ASCII 코드(DB) : ascii(문자or숫자)
ord('a')  # 97
ord('A')  # 65
char = input('소문자: ')

if ord(char) >= 97 and ord(char) <= 122:
    print(chr(ord(char) - 32))
else:
    print('잘못입력하셨습니다!')

# 16 잔돈 계산하는 프로그램
cost = 12340  # 비용
money = 50000  # 지불금액
charge = money - cost  # 잔돈

print(f'''
지불금액은 {cost:,d}원 이고,
받은금액은 {money:,d}원 이고,
잔금은 {charge:,d}원 입니다. ''')  # 37660

w50k = charge // 50000
charge = charge % 50000

w10k = charge // 10000
charge = charge % 10000

w5k = charge // 5000
charge = charge % 5000

w1k = charge // 1000
charge = charge % 1000

w5m = charge // 500
charge = charge % 500

w1m = charge // 100
charge = charge % 100

w50 = charge // 50
charge = charge % 50

w10 = charge // 10
charge = charge % 10

print(f'''
50,000원권은 {w50k}장,
10,000원권은 {w10k}장,
5,000원권은 {w5k}장,
1,000원권은 {w1k}장,
500원은 {w5m}개,
100원은 {w1m}개,
50원은 {w50}개,
10원은 {w10}개 입니다
''')

# 33 신용카드정보 판별
card = input('신용카드번호는? : ')
result = ''

if card[:2] == '35':
    card = card[2:]
    if card == '6317':
        result = 'NH농협JCB카드'
    elif card == '6901':
        result = '신한JCB카드'
    elif card == '6912':
        result = 'KB국민JCB카드'
    else:
        result = '잘못된 카드번호입니다'
elif card[:1] == '4':
    card = card[1:]
    if card == '04825':
        result = '비씨VISA카드'
    elif card == '38676':
        result = '신한VISA카드'
    elif card == '57973':
        result = 'KB국민VISA카드'
    else:
        result = '잘못된 카드번호입니다'
elif card[:1] == '5':
    card = card[1:]
    if card == '15594':
        result = '신한MASTER카드'
    elif card == '24353':
        result = '외환MASTER카드'
    elif card == '40926':
        result = 'KB국민MASTER카드'
    else:
        result = '잘못된 카드번호입니다'
else:
    result = '잘못된 카드번호입니다'

print(result)




# 48
# 현재 수지의 통장잔액이 25,000원이다. 은행이자가 연 6%일 때,
# 몇 년이 지나야 통장잔액이 지금의 2배를 넘는지 구하라.
balance = 25000
limit = balance * 2

for x in range(100):
    if balance > limit: break
    balance *= 1.06

print(x)

# 48 함수
balance = int(input('통장잔액: '))
interest = float(input('연이율: '))

def compute_interest (balance, interest):
    limit = balance * 2

    cnt = 0
    while True:
        if balance > limit: break
        balance *= (interest/100 + 1)
        cnt += 1

    print(f'{cnt}년부터 통장잔액의 2배가 넘음, 잔액={balance:,.0f}원')

compute_interest(balance, interest)

# 48 반환
def compute_interest2 (balance, interest):
    limit = balance * 2

    cnt = 0
    while True:
        if balance > limit: break
        balance *= (interest/100 + 1)
        cnt += 1
    return cnt, balance

compute_interest2(25000, 6)

print(f'{cnt}년부터 통장잔액의 2배가 넘음, 잔액={balance:,.0f}원')

# 잔돈 계산 프로그램 함수
def compute_charge(cost, money):
    charge = money - cost  # 잔돈

    w50k = charge // 50000
    charge = charge % 50000

    w10k = charge // 10000
    charge = charge % 10000

    w5k = charge // 5000
    charge = charge % 5000

    w1k = charge // 1000
    charge = charge % 1000

    w5m = charge // 500
    charge = charge % 500

    w1m = charge // 100
    charge = charge % 100

    w50 = charge // 50
    charge = charge % 50

    w10 = charge // 10
    charge = charge % 10

    print(f'''
    50,000원권은 {w50k}장,
    10,000원권은 {w10k}장,
    5,000원권은 {w5k}장,
    1,000원권은 {w1k}장,
    500원은 {w5m}개,
    100원은 {w1m}개,
    50원은 {w50}개,
    10원은 {w10}개 입니다
    ''')

compute_charge(31450, 42000)



# 51
print(f'\t\t\t Multiplication Table')
print(f'\t\t 1\t 2\t 3\t 4\t 5\t 6\t 7\t 8\t 9')
print('-'*41)

for x in range(1, 10):
    print(f'{x} | ', end='')
    for y in range(1, 10):
        print(f'\t{x*y:2d}', end='')
    print()


# p.110
# 1
lst = [10, 1, 5, 2]
result = lst * 2    # 단계1
result.append(lst[0]*2) # 단계2
result2 = result[1::2]  # 슬라이싱 사용

# 2 A형
import random as rnd
list = []
size = int(input('리스트 크기: '))

for i in range(size):
    # val = int(input(f'{i+1} 번째 값: '))
    val = rnd.randint(1, 100)
    list.append(val)

print(f'리스트 크기 : {len(list)}')

# 2 B형
import random as rnd
list = []
size = int(input('리스트 크기: '))

for i in range(size):
    val = rnd.randint(1, 100)
    list.append(val)

key = int(input('찾을 값: '))

for j in list:
    if j == key: print('Yes')
else: print('No')

# p.111 3-A
message = ['spam', 'ham', 'spam', 'ham', 'spam']
dummy = []

for msg in message:
    val = 0
    if msg == 'spam': val = 1
    dummy.append(val)

print(dummy)

# 3-B
spam_list = []
for msg in message:
    if msg == 'spam':
    spam_list.append(msg)

print(spam_list)

for ix, s in enumerate(s1): # enumerate 사용한 조회??
    print(f'{ix}번째 요소, {s}')

# 1 ~ 45사이 임의의 숫자 6개를 생성 ( 중복X )
# list, remove   뽑는 확률이 공평함
import random as rnd

lotto = list(range(1,46))   # 로또 초기화
mykey = []

size = len(lotto) - 1
for _ in range(6):
    ix = rnd.randint(0, size) # 리스트 인덱스를 난수로 생성
    key = lotto[ix]
    # print(ix, key)    # 확인용
    lotto.remove(key)   # 비복원 추출 ★★
    mykey.append(key)
    size -= 1

print(mykey)

# list, remove 뽑는 확률이 공평하지 않음
mykey = []

while True:
    if len(mykey) == 6: break
    key = rnd.randint(1, 45)
    if key not in mykey:
        mykey.append(key)

print(mykey)

# set    뽑는 확률이 공평하지가 않음
mykey = set([])

while True:
    if len(mykey) == 6: break
    key = rnd.randint(1, 45)
    mykey.add(key)

print(mykey)



# employees.csv를 이용해서 사원정보를 입력하면
# list에 각각 저장하는 코드를 작성하세요
# 사번empno, 이름fname, 성lname, 이메일email,
# 입사일hdate, 직책jobid, 급여sal, 부서번호deptid
empnos = []
fnames = []
lnames = []
emails = []
hdates = []
jobids = []
sals = []
deptids = []

empno = input('사원번호: ')
fname = input('이름: ')
lname = input('성: ')
email = input('이메일: ')
hdate = input('입사일: ')
jobid = input('직책: ')
sal = input('급여: ')
deptid = input('부서번호: ')

empnos.append(empno)
fnames.append(fname)
lnames.append(lname)
emails.append(email)
hdates.append(hdate)
jobids.append(jobid)
sals.append(sal)
deptids.append(deptid)

for i in range(len(empnos)):
    print(f'{empnos[i]}, {fnames[i]}, {lnames[i]}, '
          f'{emails[i]}, {hdates[i]}, {jobids[i]}, '
          f'{sals[i]}, {deptids[i]}')
