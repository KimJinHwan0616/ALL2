## db 파일 ## : 데이터베이스 관리 소프트 웨어(RDBMS) - sqlite3, 오라클, 마리아DB
# 파이썬 : sqlite3, 오라클, 마리아DB
# 명령 프롬 포트 : sqlite3

# 파이썬 #

#       연결
# RBDMS  ←→  db 파일
#   ↓     ↗
#   ↓   ↗  연결 후 저장
#  커서 (SQL 코드 : 데이터 검색,삽입,수정,삭제)

# sqlite3 ###################################################################
# 테이블 생성, 변경, 삭제 / 데이터 검색, 삽입, 수정, 삭제 #

# 데이터 검색

# import sqlite3    # 내장 모듈 sqlit3 호출
# (연결)변수 = sqlite3.connect('저장경로/파일_이름.db')      # sqlite3 -> db 파일 연결
# (커서)변수 = (연결)변수.cursor()      # 커서 생성

# (SQL)변수 = 'select 코드'
# 커서(변수).execute( (SQL)변수 )     # select 코드를 커서를 통해 데이터베이스에서 실행

# 모두 검색 ( fetchall )
# (행)변수 = (커서)변수.fetchall()
# for row in rows:
#    print(f'{ (행)변수[0] } ··· { (행)변수[n] }' )   # 데이터 출력, (행)변수[n] : n+1열

# 한 줄씩 검색 ( fetchone )
# while True:
#   (행)변수 = (커서)변수.fetchone()
#   if (행)변수 == None: break                      # 읽은 행이 없으면 중단
#   print(f'{ (행)변수[0] } ··· { (행)변수[n] }' )   # 데이터 출력, (행)변수[n] : n+1열

# (커서)변수.close()    # 커서 종료
# (연결)변수.close()    # sqlite3 -> db 파일 연결 종료


# naverDB.db 파일에서 product 테이블 검색 ( fetchall/fetchone )
# fetchall
import sqlite3

con = sqlite3.connect('c:/python/naverDB.db')
cur = con.cursor()

sql = 'select * from product'

cur.execute(sql)

rows = cur.fetchall()
for row in rows:
    print( f'{row[0]} | {row[1]} | {row[2]}\n' )
#    print( f'{row[0]} | {row[1]} | {row[2]} | {row[3]}' )  # 4열까지 있군

cur.close()
con.close()

# fetchone
import sqlite3

con = sqlite3.connect('c:/python/naverDB.db')
cur = con.cursor()

sql = 'select * from product'

cur.execute(sql)

while True:
    row = cur.fetchone()
    if row == None: break
    print( f'{row[0]} / {row[1]} / {row[2]}\n' )    # 3열까지만 줄바꿈으로 출력

cur.close()
con.close()

# 데이터 삽입

# import sqlite3    # 내장 모듈 sqlit3 호출
# (연결)변수 = sqlite3.connect('저장경로/파일_이름.db')      # sqlite3 -> db 파일 연결
# (커서)변수 = (연결)변수.cursor()      # 커서 생성

# (SQL)변수1 = 'insert into 테이블 values (?,?,? ..., ?)'     # 삽입할 데이터 조건값 지정
# (SQL)변수2 = (속성1_데이터, 속성2_데이터, ... 속성n_데이터)      # 삽입할 데이터
# 커서(변수).execute( (SQL)변수1, (SQL)변수2 )       　         # insert 코드를 커서를 통해 데이터베이스에서 실행
# (연결)변수.commit()                                         # 삽입한 데이터를 데이터베이스에 저장

# (커서)변수.close()    # 커서 종료
# (연결)변수.close()    # sqlite3 -> db 파일 연결 종료

import sqlite3

con = sqlite3.connect('c:/python/naverDB.db')
cur = con.cursor()

sql = 'insert into product values (?, ?, ?, ?)'
data = ('p0004', '책상', 5000, 7)

cur.execute(sql, data)
con.commit()

cur.close()
con.close()

# 데이터 수정

# import sqlite3    # 내장 모듈 sqlit3 호출
# 속성1 = input(' ')      # 수정할 데이터 입력
# ...
# 속성n = input(' ')      # 수정할 데이터 입력

# (연결)변수 = sqlite3.connect('저장경로/파일_이름.db')      # sqlite3 -> db 파일 연결
# (커서)변수 = (연결)변수.cursor()      # 커서 생성

# (SQL)변수1 = 'update 테이블 set 속성1 = :데이터, ... 속성n = :데이터 [where 조건]'    # 수정할 데이터 조건값 지정
# (SQL)변수2 =
#   { '속성1_데이터':속성1_데이터, ..., '속성n_데이터':속성n_데이터 }         # 수정할 데이터
# 커서(변수).execute( (SQL)변수1, (SQL)변수2 )       　                  # update 코드를 커서를 통해 데이터베이스에서 실행
# (연결)변수.commit()                                                   # 수정한 데이터를 데이터베이스에 저장

# (커서)변수.close()    # 커서 종료
# (연결)변수.close()    # sqlite3 -> db 파일 연결 종료

# naverDB.db 파일에 저장된 product 테이블에서
# 제품코드(pcode)가 p0002인 제품명(pname)을 모니터, 가격(price)을 30으로 입력받아서 수정

import sqlite3

code = input('제품코드: ')
name = input('수정할 제품명: ')
price = int(input('수정할 가격: '))

con = sqlite3.connect('c:/python/naverDB.db')
cur = con.cursor()

sql = 'update product set name = :name, price = :price \
        where code = :code'
data = {'code':code, 'name':name, 'price':price}

cur.execute(sql, data)
# print(f'{cur.rowcount}행 수정') # 확인용
con.commit()

cur.close()
con.close()

# 데이터 삭제
# import sqlite3        # 내장 모듈 sqlit3 호출
# 속성 = input(' ')      # 삭제할 데이터 입력

# (연결)변수 = sqlite3.connect('저장경로/파일_이름.db')      # sqlite3 -> db 파일 연결
# (커서)변수 = (연결)변수.cursor()      # 커서 생성

# (SQL)변수1 = 'delete from 테이블 where 속성 = ?]'             # 삭제할 데이터 조건값 지정
# (SQL)변수2 = [속성]                                         # 삭제할 데이터
# 커서(변수).execute( (SQL)변수1, (SQL)변수2 )       　         # delete 코드를 커서를 통해 데이터베이스에서 실행
# (연결)변수.commit()                                         # 수정한 데이터를 데이터베이스에 저장

# (커서)변수.close()    # 커서 종료
# (연결)변수.close()    # sqlite3 -> db 파일 연결 종료

# naverDB.db 파일에 저장된 product 테이블에서
# 제품코드(pcode)가 p0001인 제품명(pname)을 입력받아서 삭제
import sqlite3

code = input('삭제 제품명: ')

con = sqlite3.connect('c:/python/naverDB.db')
cur = con.cursor()

sql = 'delete from product where code = ?'
data = [code]
cur.execute(sql, data)

# print(f'{cur.rowcount}행이 제거')  # 확인용
con.commit()

cur.close()
con.close()

# 오라클 ###################################################################
# db 파일과 오라클 전자지갑 파일은 반드시 같은 폴더에 넣고 실행

#   패키지 설치
#   pip install cx_Oracle

# import cx_Oracle    # 외장 모듈 cx_Oracle 호출
# import os           # 외장 모듈 os 호출
# os.cx_Oracle('NLS_LANG', '.UTF8')     # 데이터 한글 파일 처리

# cx_Oracle.init_oracle_client(lib_dir=r'저장경로')      # 오라클 전자지갑 폴더 저장 위치
# 저장경로(예) - c:/java/oracle_cloud/network/admin

# (연결)변수 = cx_Oracle.connect(user='사용자', password='비밀번호', dsn='TNS 이름')   # 오라클(TNS)
# (커서)변수 = (연결)변수.cursor()      # 커서 생성

# (SQL)변수 = 'SQL 코드'
# 커서(변수).execute( (SQL)변수 )     # 코드를 커서를 통해 데이터베이스에서 실행

# (커서)변수.close()    # 커서 종료
# (연결)변수.close()    # 오라클 -> db 파일 연결 종료

# mariaDB ###################################################################
#   패키지 설치
#   pip install pymysql

# import pymysql    # 외장 모듈 pymysql 호출

# (연결)변수 = pymysql.connect(
#       host='서버 호스트',
#       database='데이터베이스 이름',
#       user='사용자', password='비밀번호', charset='utf8')   # mariaDB
# (커서)변수 = (연결)변수.cursor()      # 커서 생성

# (SQL)변수 = 'SQL 코드'
# 커서(변수).execute( (SQL)변수 )     # 코드를 커서를 통해 데이터베이스에서 실행

# (커서)변수.close()    # 커서 종료
# (연결)변수.close()    # mariaDB -> db 파일 연결 종료

###################################################################
