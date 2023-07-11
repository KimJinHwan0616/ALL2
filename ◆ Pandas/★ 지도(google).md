```
#   패키지 설치
#   pip install gmaps
#   pip install googlemaps
import gmaps
import googlemaps as ggmaps

#import json
#import webbrowser
#import re

from ipywidgets.embed import embed_minimal_html
from IPython.display import HTML, IFrame

key = 'AIzaSyDsM6HmHC89ZP-3NXJShVxWGnW8Sttebks'   # geocoding API 키

ggmap = ggmaps.Client(key=key)  # googlemaps(좌표) 초기화
gmaps.configure(api_key=key)    # gmaps(지도) 초기화
```

## 좌표
lat: 위도, lng: 경도
```angular2html
info = ggmap.geocode('주소', language='ko')
info[0]['geometry']['location']
```

## 지도
>```
>figure = gmaps.figure(
>    [map_type='종류']          # 종류
>    [, layout=style]           # 스타일
>    [, center=(위도, 경도)]     # 중심
>    [, zoom_level=숫자]         # 줌
>    )
>
>embed_minimal_html( '[상위폴더/]파일_이름.html', views=[figure] )   # 저장
>
>IFrame(src='[상위폴더/]파일_이름.html', width=가로, height=세로)    # 미리보기
>```
>### 종류
>``HYBRID`` : 위성, 도로, 심볼<br>
>``TERRAIN`` : 도로<br>
>``SATELLITE`` : 위성

+ ### 마커
  ```
  location = [(위도, 경도)]         # 좌표
  content = [내용]                 # 내용
  
  map_marker = gmaps.marker_layer(
      location, info_box_content=content)
  
  figure.add_layer(map_marker)     # 마커 객체 추가
  ```

+ ### 심볼
  ```
  location = [(위도, 경도)]         # 좌표

  map_symbol = gmaps.symbol_layer(
      location               # 좌표
      , fill_color='색깔'     # 색깔
      , stroke_color='색깔'   # 테두리 색깔
      , scale=크기            # 크기
      )
  
  figure.add_layer(map_symbol)     # 심볼 객체 추가
  ```

+ ### 단계구분도
  구역 음영처리 지도
  ```
  map_geojson = gmaps.geojson_layer(
      구역데이터               # 좌표
      , fill_color='색깔'     # 색깔
      , stroke_color='색깔'   # 테두리 색깔
      , fill_opacity=투명도   # 투명도(0~1)
      )
  
  figure.add_layer(map_geojson)     # 단계구분도 객체 추가
  ```
  + 컬러맵