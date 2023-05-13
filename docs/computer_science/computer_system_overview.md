---
layout: default
title: Computer System Overview
parent: Operating System
grand_parent: Computer Science
permalink: /docs/Computer_science/operationg_system/computer_system_overview
---

Computer System Overview
{: .fs-7 .fw-700 }

**운영체제는 컴퓨터 시스템을 구동시키는 소프트웨어**이다. 따라서 운영체제에 들어가기에 앞서, 컴퓨터 시스템은 무엇이고 기본적으로 동작하는 방식은 어떻게 되는지 알아보고자 한다. 

----

Basic Elements
{: .fs-6 .fw-600 }

컴퓨터 시스템을 구성하는 기본 components는 다음과 같다.
* Processor(CPU)
* Main Memory: 휘발성(전기가 나가면 data가 사라진다)이며 무척 빠르기 때문에 주기억 장치로 사용한다.
* System bus: CPU와 Memory, I/O module들이 서로 데이터를 주고 받는 통로역할을 한다.
* I/O modules: 입출력 모듈로 2차 저장장치(ex.하드디스크), communications equipment(ex.모니터), terminals 등이 있다.
 
**Processor(CPU)**
CPU는 다음과 같은 기본 세가지 components로 구성되어있다.
* ALU: Arithmetic and Logical Unit. 덧셈, 뺄셈 같은 두 숫자의 산술연산과 논리합, 논리곱 같은 논리연산을 계산하는 디지털 회로이다.
* CU: Control Unit. CPU자체를 제어한다. 
* Register: CPU안에서 데이터를 저장하는 저장장치.

이 중에서 특히 register

I/O를 어떻게 CPU가 할까? (CPU는 memory address로 memory에만 접근하고, I/O에 대해선 전혀 모를텐데)
CPU는 사실 memory를 거친다!
모든 I/O device는 device안에 그 device를 제어하는 controller가 있다. 여기엔 조그마한 buffer가 있는데, 그 buffer의 주소를 memory의 특정

