># 텐서플로, 케라스
>딥러닝 파이썬 라이브러리
> 
>### 모델 - 저장, 불러오기
```
#   패키지 설치
#   pip install tensorflow
#   pip install keras

import tensorflow as tf
from tensorflow import keras
tf.config.experimental.enable_op_determinism()

# tf.__version__       # tensorflow 버전
# keras.__version__    # keras 버전
```

## 모델 저장

+ ### 전체
    ```
    model.save('모델 이름.h5')
    ```

+ ### 가중치＆절편
    ```angular2html
    model.save_weights('모델 이름.h5') 
    ```

## 모델 불러오기

+ ### 전체
    ```angular2html
    model = keras.models.load_model('모델 이름.h5')
    ```

+ ### 가중치＆절편
    ```angular2html
    model.load_weights('모델 이름.h5') 
    ```