import pandas as pd
import numpy as np

# 데이터프레임 생성
city = ['서울', '서울', '서울', '부산', '부산', '부산', '인천', '인천']
year = [2015, 2010, 2005, 2015, 2010, 2005, 2015, 2010]
pop = [9904312, 9631482, 9762546, 3448737, 3393191, 3512547, 2890451, 263203]
region = ['수도권', '수도권', '수도권', '경상권', '경상권', '경상권', '수도권', '수도권']

data = {'도시':city, '년도':year, '인구수':pop, '지역':region}
df = pd.DataFrame(data, index=range(1,9))

# 데이터프레임 저장
df.to_csv('./data/city', index=False, sep=',')

# 정수 천단위 출력 설정 #
pd.set_option('styler.format.thousands', ',')
pd.options.display.float_format = '{:,.2f}'.format

###################################################################
# 지역별 연도별 평균 인구수 - 일반
df.groupby(['지역', '년도'])['인구수'].mean()
# 도시/연도별 총 인구수 - 피봇
df.pivot_table('인구수', ['도시', '년도'])

# 도시별 평균 인구수 - 일반
df.groupby('도시')['인구수'].mean()
# 도시별 평균 인구수 - 피봇
df.pivot_table('인구수', '도시', aggfunc='mean')

# 지역별 연도별 평균 인구수 - 주피터 스타일
b = df.groupby(['지역', '년도'])['인구수'].mean()
b.reset_index().style.format({'인구수':'{:,}'})

