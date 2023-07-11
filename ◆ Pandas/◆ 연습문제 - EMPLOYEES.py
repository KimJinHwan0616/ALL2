# 데이터 검색(SQL)
import pandas as pd
import pandasql as pql

emp = pd.read_csv('./data/EMPLOYEES.csv')
dept = pd.read_csv('./data/DEPARTMENTS.csv')

df = pd.DataFrame(emp)
pql.sqldf('select * from emp')

###################################################################
# 직책별 사원수 내림차순 조회 - 일반
emp.groupby(['JOB_ID'])['FIRST_NAME'].count().sort_values(ascending=False)
# 직책별 사원수 내림차순 조회 - 피봇
emp.pivot_table('FIRST_NAME', 'JOB_ID', aggfunc='count')\
    .sort_values(by='FIRST_NAME', ascending=False)

# EMPLOYEES + DEPARTMENTS 연결(join)
ed = emp.merge(dept, on='DEPARTMENT_ID')
ed
