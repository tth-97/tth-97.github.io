---
layout: default
title: mass-spring-damper system
parent: Physics
nav_order: 1
---

mass-spring-damper system
{: .fs-7 .fw-700 }

---

Set-Up
{: .fs-6 .fw-500 }

질량이 m1, m2인 두 물체가 linear spring과 linear viscous damper에 연결되어 있다. linear spring의 질량은 무시할 수 있을 정도로 작고 rest length는 r0이며, spring constant와 damping constant는 각각 k, c이다.
   
* rest length = r0
* spring constant = k
* damping constant = c
   
또한 두 물체의 position vector는 r1, r2이고 velocity vector는 r1', r2'이다.

* position vector of m1 = r1
* position vector of m2 = r2
* velocity vector of m1 = r1'
* velocity vector of m2 = r2'

이때 각 물체에 작용하는 internal force F1과 F2=-F1를 구해보자.
   
----

Spring force
{: .fs-6 .fw-500 }

우선 linear spring에 의해 발생하는 spring force에 대해 알아보자. 정지해 있는 spring을 오른쪽으로 당기다 놓으면, spring은 왼쪽으로 다시 돌아갈 것이다. Spring을 아래쪽으로 당기다 놓으면, spring은 위쪽으로 다시 돌아갈 것이다. spring이렇듯 spring for 원래 있었던 위치로 복원하려는 힘을 ㄱ

$$ F_{spring}\ =\ -k[|r2-r1|-r0] \frac{r2-r1}{|r2-r1|} $$

```python
def apply_mass_spring_damper_forces(self):
    
    anchorPosition, bobPosition = get_spring_position() # r2, r1
    
    springVector = bobPosition-anchorPosition # r2-r1
    currentLength = torch.norm(springVector)
    unitSpringVector = springVector / currentLength
    
    deformation = currntLength - restLength # currentLength-r0
    
    springForce = -1 * springConstant * deformation * unitVector      
```

---

Damping force
{: .fs-6 .fw-500 }

$$ F_{damper} = -c[ \frac{r2^{T}-r1^{T}}{|r2-r1|} (r2'-r1') ] \frac{r2-r1}{|r2-r1|} $$


