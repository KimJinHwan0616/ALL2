https://076923.github.io/posts/Python-opencv-3/
># OpenCV *(OPEN source Computer Vision library)*
>컴퓨터 비전 라이브러리
>
>### 열기, 색상, 출력
```angular2html
#    pip install opencv-python

import cv2
import matplotlib.pyplot as plt
import numpy as np
```
---

## 열기
`cv2.imread('경로', flags = cv2.색깔)`
```
# 컬러 → 컬러 (OpenCV → Matplotilb) ★
img_cv2 = cv2.imread('경로', flags = cv2.IMREAD_UNCHANGED)
img = cv2.cvtColor(이미지_변수, cv2.COLOR_BGR2RGB)
```

+ ### 색깔
  >원본: `IMREAD_UNCHANGED`
  >
  >컬러 *(BRG)*: `IMREAD_COLOR`
  >
  >흑백: `IMREAD_GRAYSCALE`
  

## 색상
OpenCV(B,G,R) → Matplotilb(R,G,B)
```angular2html
cv2.cvtColor(이미지_변수, cv2.색깔)
```

+ ### 색깔
  >컬러 → 컬러 *(OpenCV → Matplotilb)* ★
  >```
  >COLOR_BGR2RGB
  >```
  >
  >컬러 → 흑백
  >```
  >COLOR_BGR2GRAY
  >```
  >
  >흑백 → 컬러
  >```
  >COLOR_GRAY2RGB
  >```

## 출력
```angular2html
plt.imshow(이미지_변수)
plt.show()
```
```
print(image.shape)
plt.imshow(cv2.cvtColor(이미지_변수, cv2.COLOR_BGR2RGB))
plt.show()
```
