# 클래스 종류 #
# 데이터 저장
# VO(Value Object) - ★getter★
# DTO(Data Transfer Object) - getter/setter
# BO(Business Object)

# 데이터 처리 기능
# DAO(Data Access Object) - Create/Read/Update/Delete

# 성적처리 프로그램 VO/DAO 클래스
# VO #
class ScoreVO():
    def __init__(self, name, kor, eng, mat):
        self.name = name
        self.kor = kor
        self.eng = eng
        self.mat = mat
        self.__tot = 0
        self.__avg = 0.0
        self.__grd = '가'

    def __str__(self):
        result = f'{self.name} {self.kor} {self.eng} {self.mat} ' \
                 f'{self.__tot} {self.__avg:.1f} {self.__grd}'
        return result

    @property
    def tot(self):
        return self.__tot

    @tot.setter
    def tot(self, tot):
        self.__tot = tot

    @property
    def avg(self):
        return self.__avg

    @avg.setter
    def avg(self, avg):
        self.__avg = avg

    @property
    def grd(self):
        return self.__grd

    @grd.setter
    def grd(self, grd):
        self.__grd = grd

# DAO #
class ScoreService():
    def ReadScore(self):
        name = input('이름: ')
        kor = int(input('국어: '))
        eng = int(input('영어: '))
        mat = int(input('수학: '))

        return ScoreVO(name, kor, eng, mat)         # VO 객체 생성

    def ComputeScore(self, a):
        a.tot = a.kor + a.eng + a.mat
        a.avg = a.tot / 3
        if a.avg >= 90: a.grd = '수'
        elif a.avg >= 80: a.grd = '우'
        elif a.avg >= 70: a.grd = '미'
        elif a.avg >= 60: a.grd = '양'

score = ScoreService()                        # DAO 객체 생성

read = score.ReadScore()                      # ReadScore 메서드 실행( VO에 정보 저장 )
score.ComputeScore(read)                      # ComputeScore 메서드 실행

print(read)                                   # 확인용

