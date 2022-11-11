---
layout: default
title: 1.Beginning
parent: Game Programming in C++
grand_parent: Game
permalink: /docs/game/game_programming_in_cpp/1_beginning
---

Beginning
{: .fs-7 .fw-700 }

이 포스터는 산자이 마드하브의 저서 [Game Programming in C++]을 참고하여 작성되었다.  
    
1장에서는 게임 루프, 게임의 시간 경과에 따른 업데이트 방법, 게임 입출력의 기본 사항 등 모든 실시간 게임에 숨어 있는 핵심 개념을 다룬다. 또한 고전 게임 퐁(Pong)을 구현하는 방법을 살펴본다.   
   
게임 루프
{: .fs-6 .fw-500 }
---
**게임 루프(game loop)**는 전체 게임 프로그램의 전반적인 흐름을 제어하는 루프다. 다른 루프와 마찬가지로 게임 루프는 반복마다 수행하는 코드가 있으며, 루프 조건을 갖고 있다. 게임 루프는 플레이어가 게임 프로그램을 종료하지 않는 한 계속해서 루프를 반복한다.   
게임 루프 각각의 반복을 **프레임(frame)**이라 부른다. 게임이 초당 60프레임으로 실행된다는 것은 게임 루프가 초당 60번의 반복을 완료한다는 걸 뜻한다.   
   
상위 수준에서 게임은 각 프레임마다 다음 단계를 수행한다.
1. 입력을 받는다.
2. 게임 세계를 갱신한다.
3. 출력을 만든다.  

입력 처리는 키보드나 마우스, 컨트롤러 같은 여러 디바이스의 입력을 감지할 뿐만아니라 인터넷을 통한 데이터나(온라인 멀티플에이어 모드를 지원하는 게임) GPS 정보 등을 받을 수 있다.   
게임 세계 갱신은 게임 세계의 모든 오브젝트를 거치면서 필요에 따라 게임 오브젝트를 갱신한다는 걸 뜻한다.  
출력 생성은 그래픽뿐만 아니라 오디오, 포스 피드백(컨트롤러 진동 등), 인터넷을 통한 데이터 등을 출력하는 단계이다.   
   
다음은 팩맨 게임의 루프 의사코드이다.
```cpp
void Game::RunLoop()
{
while (!mShouldQuit)
{
//입력처리
JoystickData j = GetJoystickData();

//게임 세계 갱신
UpdatePlayerPosition(j);
for (Ghost& g : mGhost) {
if (g.Collides(player))
//유령과 충돌하는 팩맨 처리
else
g.Update();  //유령은 프로그램이 제어하는 완전한 AI이므로 이들 또한 로직을 갱신
}
//팩맨이 먹는 알갱이 처리
//...

//출력 생성
RenderGraphics();
RenderAudio();
}
}
```

