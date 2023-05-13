---
layout: default
title: tmux with singularity
parent: Etc
grand_parent: Tips
permalink: /docs/tips/etc/tmux with singularity/
---

tmux
{: .fs-7 .fw-700 }

[tmux](https://github.com/tmux/tmux)는 terminal multiplexer로, 하나의 화면에서 여러 개의 터미널을 생성하고 액세스 및 제어할 수 있도록 해준다. 이때 tmux는 화면에서 분리되어 **background에서 계속 실행**될 수 있으며 나중에 다시 연결할 수도 있다.    
   
SSH 등을 사용하여 원격 접속을 할 때 일반적으로 SSH 세션이 중단되면 그 세션에서 foreground로 실행중인 프로그램도 중단되는 문제점이 있다. 만약 tmux를 사용하여 background에서 프로그램을 실행한다면 SSH 세션이 닫혀도 작업이 중단되지 않고 실행될 수 있을 것이다.

---

설치
{: .fs-6 .fw-500 }

Debian 또는 Ubuntu에서 Binary packages로 설치하는 명령어이다.

```yaml
$ sudo install tmux 
```

이 외의 Platform은 [깃허브](https://github.com/tmux/tmux#installation)에 잘 나와있다.

---

사용방법
{: .fs-6 .fw-500 }


* 실행

```yaml
$ tmux 
```


* Given $$x \in \mathbb{R}^{n_{x}}$$ we want to estimate $$\hat{y} = P(y \mid x)$$ 
  * we want $$\hat{y}$$ to be probalility: $$ 0 \le \hat{y} \le 1 $$
* (Model) Parameters: $$w \in \mathbb{R}^{n_{x}}$$,\ $$b \in \mathbb{R}$$
* (Model) Output: $$ \hat{y} = \sigma(w^{T}x + b) $$
  * Sigmoid function $$ \sigma(z) = \frac{1}{1+e^{-z}}$$, 독립 변수가 $$\left( -\infty, \infty \right)$$의 어느 숫자이든 상관 없이 종속 변수 또는 결과 값이 항상 범위 $$ \left[ 0, 1 \right] $$ 사이에 있도록 한다.   
    ![sigmoid_function](../../../../assets/images/sigmoid_function.png){: width="60%" height="60%"}
* **The goal of logistic regression: try to learn the parameters $$w$$ and $$b$$ so that $$\hat{y}$$ becomes a good estimate of the probability of $$y$$**
* Notational convention
  * $$x_{0} = 1,\ x \in \mathbb{R}^{n_{x}}$$ ,
     
  * $$ \theta = \begin{bmatrix} \theta_{0} \\ \theta_{1} \\ \vdots \\ \theta_{n_{x}} \end{bmatrix} $$, $$\theta_{0} = b,\ \theta_{1} ... \theta_{n_{x}} = w $$ ,
     
  * $$ \hat{y} = \sigma(\theta^{T}x) $$ .

Logistic regression은 Linear combination으로 weigthed sum을 한 기존의 Linear regression 수식에 sigmoid 함수를 적용하여 출력값을 모델링한 것으로도 볼 수 있다. 이렇게 하면 Linear regression의 hyper plane으로는 fitting시키지 못하는 함수를 fitting할 수 있다.

![linear_vs_logistic](../../../../assets/images/linear_vs_logistic.png){: width="90%" height="90%"}

그렇다면, 어떻게 Logistic regression을 이용하여 위 그림과 같이 출력값을 잘 추정하는 모델을 만들 수 있을까?
 
----

Cost Function
{: .fs-6 .fw-500 }

* Model: $$\hat{y} = \sigma(w^{T}x + b)$$, where $$ \sigma(z^{i}) = \frac{1}{1+e^{-z^{i}}}$$ and $$z^{i} = W^{T}x^{i} + b$$
* Given training set $$ \{ (x^{1},y^{1}),\ ...,\ (x^{m},y^{m}) \}, we want $$ \hat{y}^{i} \approx y^{i}$$

{: .important-title}
> Note
>  
> Entropy
> To measure degree of impurity, Entropy = $$ \sum_{j} -p_{j} \log_{2} p_{j} $$ where $$ p_{h} $$ values of probability of class $$j$$.
> 
> 즉 Entropy는 $$p$$의 확률로 일로 일어날 이벤트를 확인했을 때, 얻게되는 정보의 양(또는 놀람의 정도, $$-\log_{2} p_{j} $$의 기댓값을 뜻한다.

위 Entropy이론에 근거하여 Logistic regression의 Loss(error) funtion은 다음과 같이 [cross-entropy loss](https://en.wikipedia.org/wiki/Cross_entropy)로 정의할 수 있다.

* cross-entropy loss: $$L(\hat{y},y) = -y\log \hat{y} -(1-y)\log (1- \hat{y})$$
  * If $$ y \rightarrow 1 : L(\hat{y},y) = -y\log \hat{y} \ \rightarrow want \log \hat{y} large \ \rightarrow want \hat{y} large \ \rightarrow want \hat{y} \approx 1 $$
  * If $$ y \rightarrow 0 : L(\hat{y},y) = -\log (1-\hat{y}) \ \rightarrow want \log (1-\hat{y}) large \ \rightarrow want \hat{y} small \ \rightarrow want \hat{y} \approx 0 $$
  
training set이 여러개라면, Loss funtion은 다음과 같이 Cost funtion(모든 training data의 Loss값의 평균)으로 나타낼 수 있다.
  
* cost funtion: $$J(w, b) = \frac{1}{m} \sum_{i=1}^m L(\hat{y}^i,y^i) = \frac{1}{m} \sum_{i=1}^m y^i \log \hat{y}^i -(1-y^i)\log (1- \hat{y}^i) $$

이제 추상적이였던 Logistic regression의 goal(try to learn the parameters $$w$$ and $$b$$ so that $$\hat{y}$$ becomes a good estimate of the probability of $$y$$)이 cost funtion $$J(w, b)$$을 최소하하는 $$w$$와 $$b$$값을 찾는 문제로 단순해졌다. 

