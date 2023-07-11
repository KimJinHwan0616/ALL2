https://drive.google.com/file/d/18Yk3n4Ldl08TiR7KM1V8UdsS0gsu6xsi/view?ts=633b8329
https://drive.google.com/file/d/1uLmtH1BrbwDTiqr5NoDSrSoXD8aPT5UU/view?ts=6344bc7e

># RPA
>�κ� ���μ��� �ڵ�ȭ(Robotic Process Automation)
>
>### ����Ű, ��Ƽ��Ƽ, ����, �ڷ���, ����, ...

---
## ����Ű
+ ``F2`` : �̸� ���桡������``Shift+F2`` : �ּ���������``Shift+F5`` : ����

---
## ��Ƽ��Ƽ
���ߵ���

+ ### ����
  >``Sequence`` : ����<br>
  >``Flowchart`` : ������<br>
  >``Assign`` : ���� ����<br>
  >~~``input Dialog`` : �Է�â~~<br>
  >``Write Line`` : ���â

+ ### ���ǹ�
  >``IF`` : Sequence<br>
  >``Flow Decision`` : Flowchart

+ ### �ݺ���
  > ``While, Do while`` : �Ϲ�<br>
  > ``For Each`` : �迭, ����Ʈ<br>

+ ### ����
  >``Excel Apllication Scope`` : �ҷ�����(�ּ�)<br>
  >��������``Write Range`` : ����<br>
  >��������``Read Range`` : �б�
  > 
  >~~``Append Range`` : ������ ���̺� �߰�~~

+ ### ������ ���̺�
  >``Build Data Table`` : ����<br>
  >``Filter Data Table`` : ���� �˻�<br>
  > 
  >``For Each Row in Datatable`` : �ݺ���<br>
  >��������``Add Data Row`` : �� �߰�(����)<br>

+ ### ������
  >``Open Browser`` : ����<br>
  >��������``Maximize Window`` : Ȯ��<br>
  > 
  >``Attach Browser`` : �Ӽ� �ν�<br>
  >��������``Navigate To`` : ����Ʈ �̵�<br>
  >��������``Close Tab`` : �ݱ�(1��)
  > 
  >``Delay`` : ����
  >
  >``Click`` : ���콺��������``Type into`` : Ű����<br>
  >``Get Password`` : �н����� ��ȣȭ<br>
  >``Send HotKey`` : ����Ű 
  >
  >``Get Text`` : �ؽ�Ʈ ����(���ڿ�)<br>
  >``Select Item`` : ��ӹڽ� ��� ����

+ ### ����ó��
  >``Element Exists`` : If / Flow Decision<br>

---
## ����
+ ### ����
  >``String`` : ���ڿ�(Str)��������``Int32`` : ����(Int)��������``Double`` : �Ǽ�(Dbl)<br>
  > 
  > ``Array of [T]`` : �迭(String[], Int32[], ...)��������``List`` : ����Ʈ��������``Dictionary`` : ����<br>
  > 
  >``Datatable`` : ǥ(DT)<br>
  > 
  >``Boolean`` : ��, ����(Bln)<br>
  > 
  >``GenericValue`` : ��� Ÿ�� �ڷ���

+ ### �ʱ�ȭ
    ```
    String = ""��������
    Int32 = 0
    Array = new [type](ũ��-1){}
    List = new List(of Type)
    Datatable = new Datatable()
    Object = Nothing
    Dictionary = new Dictionary(of String, Object)
    ```
---
## �ڷ���
+ ### ��ȯ
  ``�ڷ���.Tostring`` : ���ڿ�<br>
  ``cdbl(�ڷ���)`` : �Ǽ���������``cint(�ڷ���)`` : ����

---
## ����
+ ### ������
  �� ���ñ⿡ �ش��ϴ� UI������Ʈ�� ã�� �� �����ϴ�
  ```angular2html
  ������ - ȭ��, ���� Ȯ��
  
  �ʷϻ� - UI �� �� �� Ŭ��
  ```
+ ### ����
  ��ü������ �ν��Ͻ��� �������� �ʾҽ��ϴ�
  ```angular2html
  �Ӽ� ��� �� ĭ Ȯ��(Read Range, Bulid Data Table, ...)
  
  ���� Ȯ��(For Each Row in Datatable, Add Data Row, ...)
  ```
  

---
