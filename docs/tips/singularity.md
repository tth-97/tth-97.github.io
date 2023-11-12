---
layout: default
title: Singularity
last_modified_date: 2023-11-01 16:44
parent: Etc
grand_parent: Tips
nav_order: 2
permalink: /docs/tips/tec/singularity/
---

Singularity 
{: .fs-7 .fw-700 } 


<br/>
---

Sandbox
{: .fs-6 .fw-700}

Singularity 이미지를 sandbox(디렉터리) 형태로 변환하면 파일 시스템에 직접 접근할 수 있게 된다. 이렇게 하면 이미지 내의 파일들을 수정할 수 있다.

```yaml
# singularity 이미지를 sandbox 형태로 변환
singularity build --sandbox new-sandbox/ old.sif

# sandbox 디렉터리 내부의 파일을 원하는대로 수정
# ...

# sandbox 디렉터리를 다시 sif 형태의 이미지로 변환
singularity build new.sif new-sandbox/
```




