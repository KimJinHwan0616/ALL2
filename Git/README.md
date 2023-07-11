># VCS *(Version Control System)*
>버전 관리 시스템
>### 깃 호스팅 서비스
```
토큰
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
```
---

## 깃 호스팅 서비스 *(Git Hosting Service)*
깃 사용자 `클라우드`
```angular2html
예) 깃허브, 깃랩, 빅버킷, ...
```

+ ### 시간
  ###### <img src = 'img/시간.png'>
  ```angular2html
  파일 생성: 워킹 트리 → 스테이지 → 커밋 → 원격 저장소
  파일 수정: 스테이지 → 워킹 트리(modified) → 스테이지 → 커밋 → 원격저장소
  파일 불러오기: 원격 저장소 → 워킹 트리
  ```
  >워킹 트리 *(Working tree)*: 작업 공간
  >
  >로컬 저장소: `.git` 폴더
  >>스테이지 *(stage)*: 임시 공간 
  >>
  >>커밋 *(commit)*: `버전` ★
  >
  >원격 저장소 *(Remote Repository)*: `GitHub`

+ ### 공간
  >브랜치 *(branch)*: `버전` 복사
  >```
  ><그림>
  >
  >병합 커밋(merge commit)
  >빨리 감기(fast-forward)
  >충돌(conflict)
  >```
  >
  >포크 *(fork)*: `원격 저장소`*(업스트림)* 복사 
