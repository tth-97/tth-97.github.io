---
layout: default
title: Q-Learning with Frozen Lake Problem
parent: Reinforcement Learning
grand_parent: Artificial Intelligence
nav_order=1
permalink: /docs/artifirial_intelligence/reinforcement_learning/q_learning_frozen_lake_problem/
---
Q-Learning with Frozen Lake Problem
{: .fs-7 .fw-700 }

**강화학습(Reinforcement Learning)이란 반복적인 시도로 시행착오를 겪으며, 주어진 외부 환경으로 부터 Reward를 받고 이를 통해 Goal에 도달하는 기계 학습을 말한다.** **Q-Learning**은 강화학습 가운데 가장 널리 사용되는 기계 학습 알고리즘으로 그 식은 다음과 같다.
   
$$Q(s,a) = r(s,a) + \gamma \max_{a'} (Q(s',a'))$$
* $$Q(s,a)$$는 estimated utility function으로, State $$s$$에서 Action $$a$$를 선택하는 것이 얼마나 유리한지 그 정도를 나타낸다.
* $$Q(s,a)$$는 Action $$a$$를 선택하여 얻을 수 있는 **즉각적인 reward**와 Action $$a$$로 인해 변화된 State $$s'$$에서 얻을 수 있는 **잠재적 reward의 최대값**의 합으로 정의한다.
* $$Q(s,a)$$의 학습이 완료되면 각 Step마다 현재 State $$s$$에 대하여 평가함수  $$Q(s,a)$$를 최대화하는 Action $$a$$를 선택한다.


Frozen Lake Problem은 Safe(S)에서 출발하여 Hole(H)을 피해 Frozen(F)인 부분만 밟으며 Goal(G)에 도착하는 문제이다. H에 빠지면 reward=-1을 얻고 게임이 끝난다. 목표지점인 G에 도착하면 reward=1을 얻고 게임이 끝난다. 학습 후 S에서 G로 가는 Rout(R)을 구할 수 있다. discount factor $$\gamma$$는 0.5로 설정하였다.   
   
입력: FrozenLake_1.txt   
1 4 4 //file number, row, column   
**S**FFF   
FHFH   
FFFH   
HFF**G**   
   
출력: FrozenLake_1_output.txt   
1 4 4   
**SRR**F   
FH**R**H   
FF**R**H   
HF**RG**   

----

전체적인 흐름은 Temporal Difference방법론을 따른다. 구체적으로는 다음과 같다.

1. 한 스텝의 경험을 쌓고
2. 경험한 데이터로 $$q(s,a)$$ 테이블의 값을 업데이트하고 (정책 평가)
3. 업데이트된 $$q(s,a)$$ 테이블을 이용하여 $$\epsilon -greedy$$ 정책을 만들고 (정책 개선)
4. $$q*(s,a)$$가 수렴할 때까지 1~3 n번 반복

구현한 코드는 **환경**과 **에이전트**에 해당하는 2개의 class와 **경험을 쌓고 학습을 하는** 1개의 main함수, input file을 처리하는 부분으로 이루어져있다. output file은 main함수에서 다루었다.

```python
import random #라이브러리 import

class LakeWorld(): #환경
    #에이전트의 액션을 받아 상태변이를 일으키고, 보상을 줌

class QAgent(): #에이전트
    #4방향 랜덤 전책을 이용해 움직임

def main(file_number): #경험을 쌓고 학습을 함
    #경험 쌓는 부분: 에이전트가 환경과 상호작용하여 데이터를 축적
    #학습하는 부분: 쌓인 경험을 통해 테이블을 업데이트

if __name__=="__main__":
    #input함수 처리
```

----

코드 설명
{: .fs-6 .fw-500 }

다음은 위 class와 함수의 세부 코드이다. 그 역할을 주석으로 설명하였다.

**LakeWorld 클래스**
```python
class LakeWorld():
    def __init__(self): #에이전트의 현재 위치를 S로 초기화
        self.x = start_x
        self.y = start_y

    def step(self, a): #액션을 받아 현재 위치를 바꾸고, 그에 따른 보상을 정해줌. 내부에서 아래 4개의 move함수를 호출한다.
        if a==0:  #받은 액션 a가 0이면 self.move_up()함수 호출
            self.move_up()
        elif a==1:
            self.move_right()
        elif a==2:
            self.move_down()
        elif a==3:
            self.move_left()

        reward = self.get_reward() #보상을 정해준다.
        terminated = self.is_terminated() #에피소드가 끝났는지 확인한다.
        return (self.x, self.y), reward, terminated #다음 상태와 보상, 에피소드가 끝났는지 여부를 리턴

    def move_up(self): #현재 위치에서 한 칸 위로 
        if self.x==0: #호수 바깥으로 진행하는 액션은 무효처리
            pass      
        else:
            self.x -= 1 #호수 바깥으로 나가지 않으면 self.x의 위치를 한 칸 위로

    def move_right(self): #현재 위치에서 한 칸 오른쪽으로
    	... #생략
    	
    def move_down(self): #현재 위치에서 한 칸 오른쪽으로
        ... #생략
        
    def move_left(self): #현재 위치에서 한 칸 왼쪽으로
        ... #생략
    
    def is_terminated(self): #에이전트의 현재 위치가 종료 위치인 'H'이거나 'G'이면 True를, 그렇지 않으면 False를 리턴
        if lake[self.x][self.y]=='H' or lake[self.x][self.y]=='G':
            return True
        else:
            return False

    def get_reward(self): #에이전트의 현재 위치가 'H'이면 -1을, 'G'이면 1을, 그렇지 않으면 0을 리턴
        if lake[self.x][self.y]=='H':
            return -1
        elif lake[self.x][self.y]=='G':
            return 1
        else:
            return 0

    def reset(self): #에이전트의 현재 위치를 S로 reset
        self.x = start_x
        self.y = start_y
        return (self.x, self.y) #에이전트가 종료 상태에 도달했으면 다시 처음 상태로 돌려놓음
```

**QAgent 클래스**
```python
class QAgent():
    def __init__(self):
        self.q_table = [[[0 for c in range(col)] for r in range(row)] for depth in range(a_num)] #row=x축(세로), col=y축(가로), depth=z축(높이)인 3차원 리스트(Q테이블) 생성, 0으로 초기화 
        self.gamma = 0.5 #gamma는 0.5
        self.eps = 0.9 #epsilon은 0.9

    def select_action(self, s): #eps-greedy로 액션을 선택
        x, y = s
        coin = random.random() #0에서 1사이의 random한 수
        if coin < self.eps: #coin 0.9보다 작으면 0,1,2,3 중 랜덤하게 하나 선택
            action = random.randint(0,3)
        else: #그렇지 않으면 현재 q_table의 값을 읽어 가장 최적의 action선택
            action = 0
            action_val = self.q_table[0][x][y]
            for i in range(a_num):
                tmp = self.q_table[i][x][y]
                if tmp>action_val:
                    action_val = tmp
                    action = i
        return action #action 리턴

    def update_table(self, transition): #Q테이블 업데이트
	#transition은 상태 전이 1번을 뜻함
	#상태 s에서 a를 해서 보상 r을 받고 상태 s'에 도달했다면 (s,a,r,s')이 하나의 transition
	#TD 학습은 샘플 하나만 생기면 바로 엡데이트할 수 있다.
        s, a, r, s_prime = transition
        x, y = s
        next_x, next_y = s_prime
        
	#maxQ 구해주기
        maxQ = self.q_table[0][next_x][next_y]
        for i in range(a_num):
            tmp = self.q_table[i][next_x][next_y]
            if tmp>maxQ:
                maxQ = tmp
                
        #Q러닝 업데이트
        self.q_table[a][x][y] = r + self.gamma * maxQ
```

**main 함수**
```python
def main(file_number):
    env = LakeWorld() #LakeWorld 클래스의 인스턴스 env변수 선언, 환경을 만듦
    agent = QAgent() #QAgent 클래스의 인스턴스 agent변수 선언, 에이전트을 만듦

    for n in range(700000): #총 700000번의 에피소드 진행
        terminated = False
        s = env.reset()
        while not terminated:
            a = agent.select_action(s) #action을 선택
            s_prime, r, terminated = env.step(a) #step을 진행
            agent.update_table((s,a,r,s_prime)) #한 스텝이 끝날때마다 update_table 함수 호출
            s = s_prime
    best_action_list = agent.get_best_action_list() #현재의 Q테이블 중에 가장 best인 action을 선택
    
    #output파일처리
    if file_number==1: #현재 open되어 있는 파일의 번호가 1이면
        f = open('FrozenLake_1_output.txt', 'w') #'FrozenLake_1_output.txt'을 쓰기모드로 생성
   ...(생략) #파일 2, 3에도 동일하게 적용
    f.write(str(file_number)+" "+str(row)+ " "+str(col)+"\n") #파일의 첫번째 라인(file number, row, column)쓰기
    
    (x_, y_) = env.reset() #현재 위치를 S로 초기화
    path=[] #path를 저장할 리스트 생성
    while(best_action_list[x_][y_]!='x'): #현재 위치가 G에 도달할 때까지
        best_action=best_action_list[x_][y_] #현재 위치에서 best_action을 선택하여 준다.
        if best_action==0: #만약 현재 위치에서 best_action이 0이면
            x_ -= 1 #현재 위치를 한 칸 위로
        ... #생략 #best_action에 따라 현재 위치를 업데이트 해 준다
        path.append((x_,y_)) #path기록
    
    for i in range(row):
        for j in range(col):
	 	... #생략
                if in_path: #lake의 'F'의 좌표가 path안에 있다면 
                    f.write('R') #'R'을 출력
                    ... #생략 #그렇지 않다면 lake의 'F' 'H', 'G'을 그대로 출력
    f.close()
```

**main 함수 실행**
```python
if __name__ == "__main__":

    with open("FrozenLake_1.txt", 'r') as f1: #파일 FrozenLake_1.txt을 읽기모드로 open
        lines1 = f1.read().splitlines() #파일 FrozenLake_1.txt의 내용을 read

    ...(생략) #파일 2, 3에도 동일하게 적용

    files = { 1:lines1, 2:lines2, 3:lines3} #파일 번호와 함께 각 파일의 내용을 딕셔너리 형태로 저장
 
    for f in [1,2,3]: #각각의 파일에 대하여
        info = files[f].pop(0) #파일의 첫번째 라인(file number, row, column)을 읽어드린다.
        row = int(info.split()[1]) #row 정의
        col = int(info.split()[2]) #column 정의
        lake = files[f] #FrozenLake 정의
        for i in range(row): #각 row에 대하여
            for j in range(col): #각 column에 대하여
                if f==1: #파일 번호가 1일때
                    if lake[i][j]=='S':
                        start_x = i
                        start_y = j
		... #생략 #파일 2, 3에도 동일하게 적용
        main(f) #각각의 파일에 대해여 main함수 실행
```

