---
layout: default
title: Database Systems
parent: Database
grand_parent: Computer Science
permalink: /docs/Computer_science/database/database_systems
---

Database Systems
{: .fs-7 .fw-700 }

A **database(DB)** is a **collection of data organized in a structured way** by using **schema**. A **database management system(DBMS)** is software managing and allows users to access databases by **concurrency control**.   
즉 database는  schema를 사용한 structured data의 집합을 가리키며, 이러한 database를 동시성 제어를 통해 관리하는 software를 database management system(DBMS)이라고 한다.  
    
DBMS도  OS와같이 data를 RAM에 저장하고, OS의 file system처럼 open,read등을 통해 데이터를 관리하므로 OS가 하는 작업과 유사하다고 볼 수 있다. 하지만 DBMS는 OS와 달리 내부에 data를 저장할 때 그냥 마구잡이로 data를 집어넣지 않고, 사용자가 기술하는 모델 방식에 맞게 **구조화된 형태**의 data를 저장하고 관리한다는 것에서 큰 차이가 있다.(이를 OS가 하기엔 복잡하고 무거운 front-end part가 들어가야하기 때문에 DBMS가 담당한다.)
   
---

Schema
{: .fs-6 .fw-500 }
   
한 system에서 data를 체계적으로 관리하기 위해서는    
1. 필요한 data가 무엇인지를 파악해야하고   
2. 그 필요한 data를 어떻게 기술할 것인지를 알아야한다.    
   
이렇듯 내가 구축하고 싶어하는 system에서 저장해야 할 데이터를 기술하기 위해 필요한 concept을 **data model**이라고 하며, 주어진 data model을 사용하여 실질적으로 data를 기술해 놓은 것을 **schema**라고 한다.   
   
    
> 학생들이 자신이 수강하는 과목의 성적을 확인할 수 있는 system을 생각해보자. 이 system에서 필요한 데이터는 성적을 확인하려는 학생의 정보와 학생이 확인하려는 과목 정보, 그 과목의 수강현황 정보가 필요하다. 학생의 정보는 학생 id와 학생의 name, 학생의 gpa로 기술할 수 있고, 과목 정보는 과목 id와 과목 name, credits으로 기술할 수 있다. 수강현황은 어떤 학생이 어떤 과목을 몇 점 맞았는지를 기술하며, 앞의 두 정보의 관계를 나타낸다. 이 system의 logical한 schema는 다음과 같다.
>     
>> * **Logical Schema**
>>	* Students(sid: string, name: string, gpa: float)
>>	* Courses(cid: string, name: string, credits: int)
>>	* Enrolled(sid: string, cid: string, grade: string)
>>   
>>   
>>    
>> < Students >
>> {: .text-blue-300 } 
>>       
>> | sid   | name | gpa |   
>> |:----- |:-----|:----|   
>> | 101  | Harry| 3.6 |   
>> | [123][1]{: .btn .btn-green } | Ron  | 3.1 |   
>>   
>>                           
>> < Courses >
>> {: .text-bule-300 }                  
>>     
>> | cid   | name   | credits |   
>> |:------|:-------|:--------|   
>> | 511 | Potions| 4       |   
>> | [308][2]{: .btn .btn-blue } | Flights| 3       | 
>>   
>>    
>> < Enrolled >
>> {: .text-bule-300 }        
>>      
>> | sid   | cid   | grade | 
>> |:------|:------|:------|  
>> | [123][1]{: .btn .btn-green } | [308][2]{: .btn .btn-blue } | [A][3]{: .btn .btn-purple }     |
>   
> 이때 Students data와 Courses data를 entities라 하고, Enrolled data를 두 data의 관계를 나타내므로 relationship이라고 한다. relationship은 위와 같이 특정한 entities를 가리키고 있어야 relationship이 타당하다고 할 수 있다.

---

Concurrency control
{: .fs-6 .fw-500 }



[1]: https://media.vingle.net/images/ca_l/hud3qv7a3x.jpg
[2]: https://images.ctfassets.net/usf1vwtuqyxm/4a12VXxaO4sGmqCw8gQ0Qu/20c4c11611d9047887d4d110d13e8901/WB_F1_MadamHoochFirstFlyingLesson_Neville_00356423.jpg?w=914&q=70&fm=webp
[3]: https://media.vingle.net/images/ca_l/pw07wigfde.jpg


