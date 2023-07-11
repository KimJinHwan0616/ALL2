## 객체지향 프로그래밍(기본, 상속, 캡슐화) ##
# 생성자 : 객체를 생성함과 동시에 실행시켜주는 메소드 ( = 객체 정보 초기값 설정 )
# __init__ : 변수 / __str__ : 문자열 / ...

###################################################################
# 예 - 사칙연산(+,/) 클래스
class Calculator:                   
    # 생성자 __init__ 생성
    def __init__(self, first, second):
        self.first = first
        self.second = second

    # 사칙연산(+) 정의
    def add(self):
        result = self.first + self.second
        return result

    # 사칙연산(/) 정의
    def div(self):
        result = self.first / self.second
        return result

a = Calculator(4, 2)     # 변수 초기값 : init 생성자
print(a)                 # 문자열 초기값 : str 생성자

a.first
a.second
a.add()
a.div()

###################################################################
# 상속 # : 상위클래스로부터 클래스_변수, 메소드를 가져와 하위클래스에서 기능을 추가,변경 ( 생성자는 호출로 가져와야함 )
# 추상 클래스/추상 매서드
# super().__init/str__() : 상위클래스 생성자 호출

# class 하위클래스_이름(상위클래스_이름):
#   [def __init/str__(self):
#       super().__init/str__]
#           코드

# 예 - Calculator 클래스에 제곱 ★기능 추가★
class PowCalculator(Calculator):
    def pow(self):
        result = self.first ** self.second
        return result

b = PowCalculator(4,2)
b.pow()

# 예 - 나눗셈 오류 처리 ★기능 변경★
class SafeCalculator(Calculator):
    def div(self):
        if self.second == 0:
            return 0
        else:
            return self.first / self.second

a = Calculator(4, 0)
a.div()

# 추상 클래스 : 하위클래스에서 반드시 실행해야 하는 상위클래스의 메소드 ( 객체 생성X )
# 추상 메소드 : 내용이 없는 메소드

# from abc import ABCMeta, abstractmethod           # 모듈 abc 호출
#
# class 클래스_이름(metaclass=ABCmeta):               # 추상 클래스 생성
#       
#               ...
#
#   @abstractmethod                                 # 추상 메소드 생성
#   def 메소드1(self):
#       pass
#
#           ...
#
#   @abstractmethod
#   def 메소드n(self):
#       pass

# 예 - 스타크래프트 종족, 유닛 클래스 만들기
# 종족은 테란이고 유닛이 해병대, 의무관, 화염방사병이 있으며
# 유닛은 이름, 생명력100, 에너지100, 공격력(해병대:10, 의무관:1, 화염방사병:20)과
# 이동능력을 가지며, 유닛마다 고유능력이 한 가지씩 존재하는 종족, 유닛 클래스 생성
from abc import ABCMeta, abstractmethod

class Terran(metaclass=ABCMeta):       # 상위클래스 Terran 생성
    def __init__(self):
        self.name = self.__class__.__name__      # 클래스_이름을 변수로 사용
        self.life = 100
        self.energy = 100

    def __str__(self):
            msg = f'{self.name} {self.life} ' \
                  f'{self.energy}'
            return msg

    def move(self):
            print('지정한 위치로 이동하였습니다')

    @abstractmethod
    def attack(self):
            pass

class Marin(Terran):            # Terran 상속
    def __init__(self):
        super().__init__()      # 상위클래스 생성자 호출
        self.power = 10

    def __str__(self):
        msg = super().__str__() # 상위클래스 생성자 호출
        msg += f' {self.power}'
        return msg

    def attack(self):
        print(f'공격력 {self.power} 플라즈마 소총으로 공격중...')

class Medic(Terran):
    def heal(self):
        print('지정한 유닛을 치료중...')

class Firebat(Terran):
    def __init__(self):
        super().__init__()
        self.power = 20

    def __str__(self):
        msg = super().__str__()
        msg += f' {self.power}'
        return msg

    def attack(self):
        print(f'공격력 {self.power} 화염방사기로 공격중...')

marin = Marin()
print(marin)
marin.attack()

medic = Medic()
print(medic)
medic.heal()

firebat = Firebat()
print(firebat)
firebat.attack()

###################################################################
# 캡슐화 # : 객체의 내용을 외부로부터 숨기는 방법(은닉)
# self.__클래스_변수
# property : 은닉한 변수 가져오는 메소드
# setter : 매개변수를 은닉한 변수로 바꿔주는 메소드

# class 클래스_이름:
#   def __init__( self, 매개변수1, ..., 매개변수n )
#       self.__클래스_변수1 = 매개변수1             # 캡슐화
#               ...
#       self.__클래스_변수n = 매개변수n
#
#   @property
#   def 매개변수1(self):
#       return self.__클래스_변수1         # 은닉한 클래스_변수1 가져오기
#
#   @매개변수1.setter
#   def 매개변수1(self, 매개변수1):
#       self.__클래스_변수1 = 매개변수1     # 매개변수1 -> 은닉한 클래스_변수1
#               ...
#
#             n번 실행

# 예 - 국어, 영어, 수학 점수를 캡슐화
class Score:
    def __init__(self, kor, eng, mat):
        self.__kor = kor
        self.__eng = eng
        self.__mat = mat

    @property
    def kor(self):
        return self.__kor

    @kor.setter
    def kor(self, kor):
        self.__kor = kor

    @property
    def eng(self):
        return self.__eng

    @eng.setter
    def eng(self, eng):
        self.__eng = eng

    @property
    def mat(self):
        return self.__mat

    @mat.setter
    def mat(self, mat):
        self.__mat = mat
