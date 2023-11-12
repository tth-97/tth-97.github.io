---
layout: default
title: Quaternion to Ratation Matrix
parent: Computer Graphics
nav_order: 2
---

Quaternion to Ratation Matrix
{: .fs-7 .fw-700 }

**Quaternion**은 실수 계산 과정에서 회전 변환 행렬에 비해 안정적이고 여러 회전들의 조합을 빠르게 처리할 수 있으며 회전들 사이의 보간에 유용하다는 장점이 있지만, 2-to-1 mapping, 즉 하나의 회전을 뜻하는 값이 두개인 문제점이 있다(**q** and **-q** represent the same rotation).   

반면 **Rotataion Matrix**는 1-to-1 mapping이며 continuous하고 6D Rotational Representation을 사용하면 parameter 수도 줄일 수 있으므로, 특히 deep motion synthesis research등에 많이 사용되는 추세이댜.   
로봇의 Orientation은 주로 Quaternion으로 표현되므로(ROS 등), Quaternion값을 deep learning에 사용하기 위해 3x3 Rotation Matrix로 바꾸는 방법에 대해서 알아보자.   
    
 <br/>     
 
{: .important-title}
> Note   
>   
> **6D Rotational Representation**   
>   
> Q: Do we need all 9 numbers of Rotation matrix for learning?   
> A: NO.   
>       
> We can just taking two columns of rotation matrix. The other one can be reconstructed using **cross product**.   
> Note that this 6D representation as a network output requires an **otrhogonalization** step (e.g. Gram-Schmidt process).

<br/>


---


Convert a Quaternion to a Rotation Matrix
{: .fs-6 .fw-700 }
   
어떤 점 $$p$$를 단위 사원수 $$q=(w, x, y, z)=(w, v)$$로 회전하는 식은 다음과 같다.   
   
$$p'=p+2w(v \times p) + 2(v \times (v \times p))$$   
   
위 식을 어찌어찌 잘 풀면(Tex 문법 작성하기 귀찮다..) 다음과 같이 quaternion으로부터 rotation matrix $$M$$을 구할 수 있다.   
   
$$ M =  \begin{bmatrix}
	1-2y^{2}-2z^{2} & 2xy-2wz & 2xz+2wy \\
	2xy+2wz & 1-2x^{2}-2z^{2} & 2yz-2wx \\
	2xz-2wy & 2yz+2wx & 1-2x^{2}-2y^{2} 
	\end{bmatrix} $$     
       
 <br/>   
 
{: .highlight-title }
> Note   
>   
> **Meaning of Rotation Matrix**   
>   
> A rotation matrix defines
> - **Rotation** from a global frame to be that rotated frame or,   
> - **Orientation** of new rotated frame     

<br/>

----

Python code
{: .fs-6 .fw-700 }
   
Deep Learning에 사용될 Rotation matrix는 6D Rotational Representation을 사용하면 되므로, 앞의 두 columns값만 구하면 된다.   
   
```python
import torch

def quaternion_to_matrix(q: torch.Tensor) -> torch.Tensor:
    """
    Args:
	quaternion: quaternions with vector part first, 
		as tensor of shape (..., 4).
    Returns:
	Rotation matrices as tensor of shape (..., 3, 2).
    """

    x, y, z, w = torch.unbine(q, -1)
    two = 2.0 / (q*q).sum(-1)
    matrix = torch.stack(
            (
                1-two*(y*y-z*z),
                two*(x*y-w*z),
                two*(x*y+*w*z),
                1-two(*x*x-z*z),
                two*(x*z-w*y),
                two*(y*z+w*x),
            ),
            -1,
        )
    return matrix.reshape(q.shape[:-1] + (3, 2))
```
