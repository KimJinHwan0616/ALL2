#   패키지 설치
#   pip install gmaps
#   pip install googlemaps
import gmaps
import googlemaps as ggmaps
import pandas as pd

#import json
#import webbrowser
#import re

from ipywidgets.embed import embed_minimal_html
from IPython.display import HTML, IFrame

key = 'AIzaSyDsM6HmHC89ZP-3NXJShVxWGnW8Sttebks'   # geocoding API key: AIzaSyDsM6HmHC89ZP-3NXJShVxWGnW8Sttebks
ggmap = ggmaps.Client(key=key)  # googlemaps(좌표) 초기화
gmaps.configure(api_key=key)    # gmaps(지도) 초기화

###################################################################
# 코로나 선별진료소 지도 시각화
# pip install xlrd

covid19 = pd.read_excel('./data/선별진료소.xls', engine='openpyxl')
covid19.head()