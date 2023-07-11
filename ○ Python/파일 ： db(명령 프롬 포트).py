# 명령 프롬 포트 #

# sqlite3 ###################################################################
# db 파일과 sqlite3는 반드시 같은 폴더에 넣고 실행

# sqlite3 실행
# cd \폴더_이름   # sqlite3 위치 폴더 열기
# sqlite3       # sqlite3 프로그램 실행

# db 파일 추가&열기
# .open 파일_이름.db     # 데이터베이스 파일 열기 ( 파일이 없는 경우 새로 만들어서 생성 )

# 테이블 생성
# .schema               # 테이블 생성 코드
# pragma table_info('테이블');   # 테이블 구조

# 테이블 옵션
# .table                # 데이터베이스 테이블 목록
# .header on            # 테이블 검색 모양 변경
# .mode column          # 테이블 검색 모양 변경(추천)

# 테이블(.csv) 삽입 #
# .open 파일_이름.db
# .mode csv
# .import 테이블.csv 테이블_별칭

# naverDB.db 파일에 EMPLOYEES.csv 테이블 삽입
.open naverDB.db
.mode csv
.import EMPLOYEES.csv emp

# sqlite3 종료
.quit                 # sqlite3 종료

# naverDB 데이터베이스 파일에서
# product 테이블 생성한 후 데이터 삽입하고 테이블 검색
# code, name, price, amount
# p0001, 노트북, 110, 5
# p0002, 마우스, 3, 22

create table product(
    code varchar(10),
    name varchar(10),
    price int,
    amount int
);

insert into product values ('p0001', '노트북', 110, 5);
insert into product values ('p0002', '마우스', 3, 2);