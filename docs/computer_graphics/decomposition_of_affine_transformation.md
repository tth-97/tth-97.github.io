---
layout: default
title: Decomposition of Affine Transformation
parent: Computer Graphics
nav_order: 3
---

Deomposition of Affine Transformation
{: .fs-7 .fw-700 }

Geometry를 affine transformation matrix를 통해 변환할 때, 모든 affine transformations를 조합해서 하나의 matrix로 만들 수 있다. 예를 들어, 점 P를 점 P'로 변환하는 affine transformation matrix를 M이라고 해보자.   
   
$$ P' = M \cdot P $$
    
M은 translation, rotation, scale 등 필요한 모든 transformations가 조합된 matrix라 할 수 있다.  
    
이때 중요한 점은 각각의 transformation을 어떻게 조합해서 M을 만들지 정해야 한다는 것이다. 왜냐하면 matrix의 곱셈은 교환 법칙이 성립하지 않으므로, 변환을 가하는 순서(transform matrix를 곱하는 순서)에 따라 최종 행렬 M이 달라지기 때문이다. 그렇다면, transformations를 어떻게 조합해야 우리가 원하는 행렬 M을 얻을 수 있을까?    
   
   
---

Decomposing Transformation
{: .fs-6 .fw-700 }
   
일반적으로 최종 행렬 M은 다음과 같이 조합된다.
   
$$ M = T \cdot R \cdot S $$   
   
위의 방식을 따르면 먼저 좌표계의 축을 따라서 geometry를 scale한다. 그 후 역시 좌표계의 축을 중심으로 rotation을 하고 마지막으로 translation을 한다. 보통 이와 같은 순서로 transformations를 조합하는 것이 최종 행렬 M을 affine transform한 형태로 만들 뿐만 아니라 geometry를 우리가 원하는 크기와 위치, 방향으로 자리잡게 한다.   
   
* proof
{: .fs-5 .fw-700 .text-blue-100 }

간단히 우변과 좌변이 같다는 것을 보여줌으로써 증명할 수 있다. translation transformation matrix를 T, rotation transformation matrix를 R, scale transformation matrix를 S라고 해보자.
   
$$ T =  \begin{bmatrix}
	1 & 0 & 0 & t_{x} \\
	0 & 1 & 0 & t_{y} \\
	0 & 0 & 1 & t_{z} \\
	0 & 0 & 0 & 1 
	\end{bmatrix} 
, 
   R =  \begin{bmatrix}
	r_{11} & r_{12} & r_{13} & 0 \\
	1 & 0 & 0 & t_{x}
	0 & 1 & 0 & t_{y} \\
	0 & 0 & 1 & t_{z} \\
	0 & 0 & 0 & 1 
	\end{bmatrix} $$ 
    
       
