---
layout: default
title: TORCH.CAT
last_modified_date: 2023-10-20 12:27:24
parent: PyTorch
nav_order: 2
permalink: /docs/pytorch/cat
---

TORCH.CAT
{: .fs-7 .fw-700 }

```python
torch.cat(tensors, dim=0, *, out=None) -> Tensor
```

**Parameters**  
> **tensors**(_sequence of Tensors_) - any python sequence of tensors of the same type. Non-empty tensors provided must have the same shape, except in the cat dimension.    
> **dim**(_int, optional_): the dimension over which the tensors are concatenated   

**Keyword Arguments**
> **out**(_Tensor, optional_) - the output tensor   

<br/>
**torch.cat**은 텐서들을 특정 차원을 기준으로 이어붙이는(concatenating) 작업을 수행하는 함수이다. 데이터를 처리하거나 딥러닝 모델의 구조를 설계할 때, 두 개 이상의 텐서를 하나로 합치는 경우가 자주 있다. 이러한 경우에 torch.cat을 사용한다.
<br/>

---
 

Examples
{: .fs-6 .fw-700 }

합치고자 하는 텐서는 리스트[ ]나 튜플( )로 나타내야 하며, 어느 차원을 기준으로 텐서들을 합칠 것인지 알려줘야 한다.   
<br/>
* dim=0
{: .fs-5 .fw-700 .text-yellow-200 }

dim=0 은 첫 번째 차원을 기준으로 텐서들을 합치는 것을 의미한다. 두 개의 2x3 텐서를 첫 번째 차원을 기준으로 합치면 (2+2)x3 = 4x3 텐서가 된다.

```python
x = torch.tensor([[1,  2,  3], 
                  [4,  5,  6]])
y = torch.tensor([[7,  8,  9],
                  [10, 11, 12]])
result = torch.cat((x, y), dim=0) # tensor([[1,  2,  3], 
			          #	    [4,  5,  6],
				  #	    [7,  8,  9],
				  #	    [10, 11, 12]])
```

<br/>
* dim=1
{: .fs-5 .fw-700 .text-yellow-200 }

dim=1 은 두 번재 차원을 기준으로 텐서들을 합치는 것을 의미한다. 두 개의 2x3 텐서들을 두 번째 차원을 기준으로 합치면 2x(3+3) = 2x6 텐서가 된다.

```python
x = torch.tensor([[1,  2,  3], 
                  [4,  5,  6]])
y = torch.tensor([[7,  8,  9],
                  [10, 11, 12]])
result = torch.cat((x, y), dim=1) # tensor([[1,  2,  3,  7,  8,  9],
                                  #         [4,  5,  6, 10, 11, 12]])
```

<br/>
* 응용
{: .fs-5 .fw-700 .text-yellow-200 }

```python
# depth image
x = torch.tensor([[1,  2,  3],
                  [4,  5,  6]])
y = torch.tensor([[7,  8,  9],
                  [10, 11, 12]])

# flatten
x_flattened = x.flatten(0) # tensor([1,  2,  3,  4,  5,  6])
y_flattened = y.flatten(0) # tensor([7,  8,  9, 10, 11, 12])

# concatenating
result = torch.cat([x_flattened, y_flattened], dim=0) # tensor([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])

# reshape
result = result.view(2, 6) # tensor([[1,  2,  3,  4,  5,  6],
                           #         [7,  8,  9, 10, 11, 12]])

# 참고: concatenating + reshape의 결과는 torch.stack을 쓰면 된다
result_stack = torch.stack([x_flattened, y_flattened], dim=0) # tensor([[1,  2,  3,  4,  5,  6],
                                                              #         [7,  8,  9, 10, 11, 12]])
```
<br/>
---

참고  
PyTorch 문서: [https://pytorch.org/docs/stable/generated/torch.cat.html](https://pytorch.org/docs/stable/generated/torch.cat.html)   
