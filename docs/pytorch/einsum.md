---
layout: default
title: TORCH.EINSUM
parent: PyTorch
permalink: /docs/pytorch/einsum
---

TORCH.EINSUM
{: .fs-7 .fw-700 }

```python
torch.einsum(equation, *operands) -> Tensor
```
* equation: The subscripts for the Einstein summation.
* operands: The tensors to compute the Einstein summation of.

**torch.einsum**은 [Einstein Summation Convention](https://ko.wikipedia.org/wiki/%EC%95%84%EC%9D%B8%EC%8A%88%ED%83%80%EC%9D%B8_%ED%91%9C%EA%B8%B0%EB%B2%95)에 기반한 텐서 연산 도구이다. 이 함수를 사용하면 행렬곱셈, 행렬 전치, 대각 합, 내적 등 다양한 텐서 연산을 간결하게 표현할 수 있다. 또한 연산을 문자열로 표현하기 때문에 직관적인 파악이 가능하다. 2차원 텐서의 각 요소 벡터의 내적을 구하는 방법을 찾다가 이 연산을 처음 접했는데.. 신기하군...

---
 
Einstein Summation Convention
{: .fs-6 .fw-700}

einsum은 아인슈타인 표기법으로 연산을 나타내므로, 아인슈타인 표기법을 알 필요가 있다.   
   
아래와 같은 식을 생각해보자. 
 
$$ y = \sum_{i=1}^3 c_{i}x^{i} = c_{1}x^{1} + c_{2}x^{2} + c_{3}x^{3} $$
  
아인슈타인 표기법을 사용하면, 위 식은 다음과 같이 표현된다.  
  
$$ y = c_{i}x^{i} $$
  
여기서 상위 인덱스는 지수가 아니라 벡터의 인덱스이다. 즉, $$x^{2}$$는 x의 제곱이 아닌 x의 두 번째 구성 요소로 이해되어야 한다.  
  
몇가지 예시를 통해 Common operation에서 아인슈타인 표기법과 einsum의 사용법에 대해 살펴보자.


---

Examples
{: .fs-6 .fw-700 }
   
  
* Inner product
{: .fs-5 .fw-700 .text-blue-100 }

**Einstein:** $$ \ u \cdot v = u_{i}v^{i} $$   
**Equation:** $$ \ i, i$$ ->

> $$i$$ 인덱스가 반복되므로, $$i$$차원에 대해 합산을 수행한다.   
> -> 이후에 인덱스가 없기 때문에 결과는 스칼라이다.   

```yaml
a = torch.tensor([1, 2, 3])
b = torch.tensor([4, 5, 6])
result = torch.einsum('i,i->', a, b)  # 1*4 + 2*5 + 3*6 = 32
```
<br/>
* Matrix multiplication
{: .fs-5 .fw-700 .text-blue-100 }




---

참고  
PyTorch 문서: [https://pytorch.org/docs/stable/generated/torch.einsum.html](https://pytorch.org/docs/stable/generated/torch.einsum.html)   
Wikipedia, Einstein notation: [https://en.wikipedia.org/wiki/Einstein_notation](https://en.wikipedia.org/wiki/Einstein_notation)
