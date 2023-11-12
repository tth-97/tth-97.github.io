---
layout: default
title: Branch Strategy
parent: Git
grand_parent: Tips
nav_order: 1
permalink: /docs/tips/git/branch_strategy/
---

Branch Strategy
{: .fs-7 .fw-700 } 

**Branch strategy**란 Git에서 프로젝트를 개발할 때 다양한 브랜치(branch)를 활용하여 코드의 변경사항을 관리하는 전략을 말한다. 브랜치를 통해 여러 개발자가 동시에 다양한 기능을 개발하거나, 버그를 수정하면서도 코드의 안정성을 유지할 수 있다.   

일반적으로 사용되는 브랜치 관리 전략으로는 'Git Flow' 관리 전략과 'GitHub Flow' 관리 전략이 있다. Git Flow는 [Vincent Driessen](https://nvie.com/about/)이 제안한 브랜치 관리 전략으로, 주요 브랜치와 보조 브랜치를 통한 구조화된 방식으로 작업을 진행한다. GitHub Flow는 Git Flow에서 간소화된 브랜치 관리 전략으로, GitHub에서 직접 제안한 방법이며 Pull Request(PR)를 기반으로 진행된다.   


---

Git Flow
{: .fs-6 .fw-700 }  
<br/>

**주요 브랜치**
> * **master**: 안정적인 버전의 코드를 관리한다. 항상 배포 가능한 상태를 유지해야 한다. 새로운 버전이 준비되면 release브랜치나 hotfix브랜치에서 이 브랜치로 병합하게 된다.
> 
> * **develop**: 개발 중인 코드를 관리하며 새로운 기능이나 개선 사항들이 추가되면 이곳에 병합된다. 대부분의 개발 작업은 이 브랜치를 기반으로 진행된다. 새로운 기능이 완성되면 feature브랜치에서 이 브랜치로 병합하게 된다.

<br/>
**보조 브랜치**
> * **feature**: 새로운 기능 개발을 위한 브랜치이다. develop브랜치를 기반으로 생성되며, 기능 개발이 완료되면 develop브랜치로 병합된다. 새로운 기능이나 큰 변화가 필요할 때 develop에서 분기하여 작업을 시작하고, 완료되면 다시 develop에 병합한다.
> 
> * **release**: 새로운 프로덕션 릴리즈 준비를 위한 브랜치이다. 이 브랜치에는 버그 수정, 문서화 작업 등 릴리즈와 직접적으로 관련된 작업만을 수행한다. 릴리즈를 위한 마지막 테스트와 준비 작업이 이루어지며, 모든 준비가 완료되면 master와 develop에 병합되어 배포된다.
>
> * **hotfix**: 프로덕션에서 발생한 긴급한 버그를 수정하기 위한 브랜치이다. master브랜치를 기반으로 생성되며, 수정이 완료되면 master와 develop브랜치로 병합된다. 수정 후에는 즉시 배포된다.

<br/>
* Work Flow
{: .fs-5 .fw-700 .text-yellow-200}
<br/>
**0. 준비**

```yaml
# 레포지토리를 로컬에 클론한 후
git clone [repository_url]

# develop 브랜치를 생성 및 동기화
git checkout -b develop
git pull origin develop

# 코드 수정

# 변경 사항 커밋
git add .
git commit -m "Initial commit on develop branch"

# 브랜치를 원격 레포지토리에 푸시
git push origin develop
```
<br/>
**1. Feature 개발**

새로운 기능 개발은 **feature** 브랜치에서 시작된다. 

```yaml
git checkout -b feature/new-feature develop
```

개발이 완료되면 **develop** 브랜치로 병합(merge)한다.

```yaml
git checkout develop
git merge --no-ff feature/new-feature
git branch -d feature/new-feature
```


---

참고   
A successful Git branching model by Vincent Driessen: [https://nvie.com/posts/a-successful-git-branching-model/](https://nvie.com/posts/a-successful-git-branching-model/)
