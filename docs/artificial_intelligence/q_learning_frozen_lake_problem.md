layout: default
title: Q-Learning: Frozen Lake Problem
parent: Artificial Intelligence
nav_order: 1

---

Q-Learning: Frozen Lake Problem
{: .fs-7 .fw-700 }

강화학습(Reinforcement Learning)이란 반복적인 시도로 시행착오를 겪으며, 주어진 외부 환경으로 부터 Reward를 받고 이를 통해 Goal에 도달하는 기계 학습을 말한다. **Q-Learning**은 강화학습 가운데 가장 널리 사용되는 기계 학습 알고리즘으로 그 식은 다음과 같다.   
<math> \Q(s,a) = r(s,a) + \gamma \max_{a'} (Q(s',a'))  </math>
* <math> \Q(s,a) </math>는 estimated utility function으로, State <math> s </math>에서 Action <math> a </math>를 선택하는 것이 얼마나 유리한지 그 정도를 나타낸다.
* <math> \Q(s,a) </math>는 Action <math> a </math>를 선택하여 얻을 수 있는 **즉각적인 reward**와 Action <math> a </math>로 인해 변화된 State s'에서 얻을 수 있는 **잠재적 reward의 최대값**의 합으로 정의한다.
* <math> \Q(s,a) </math>의 학습이 완료되면 각 Step마다 현재 State <math> s </math>에 대하여 평가함수 <math> \Q(s,a) </math>를 최대화하는 Action <math> a </math>를 선택한다.





입력: FrozenLake_1.txt
1 4 4
SFFFㅇㅇㅇㄹ
FHFH
FFFH
HFFG
 

----
   
게임 루프
{: .fs-6 .fw-500 }

게임은 프로그램이 실행되는 동안 매초마다 여러 번 갱신되어야한다. 이를 위한  루프(loop)가 존재하는데 이를 **게임 루프(game loop)**라 한다. 게임 루프는 게임 클래스의 속에 있으며 게임 프로그램의 전반적인 흐름을 제어한다. 플레이어가 게임 프로그램을 종료하지 않는 한 루프는 계속해서 반복된다. 게임이 초당 60프레임으로 실행된다는 것은 게임 루프가 초당 60번의 반복을 완료한다는 걸 뜻한다.   
   
상위 수준에서 게임은 각 프레임마다 다음 단계를 수행한다.
1. 입력을 받는다.
2. 게임 세계를 갱신한다.
3. 출력을 만든다.  

입력 처리는 키보드나 마우스, 컨트롤러 같은 여러 디바이스의 입력을 감지할 뿐만아니라 인터넷을 통한 데이터나(온라인 멀티플에이어 모드를 지원하는 게임) GPS 정보 등을 받을 수 있다.   
게임 세계 갱신은 게임 세계의 모든 오브젝트를 거치면서 필요에 따라 게임 오브젝트를 갱신한다는 걸 뜻한다.  
출력 생성은 그래픽뿐만 아니라 오디오, 포스 피드백(컨트롤러 진동 등), 인터넷을 통한 데이터 등을 출력하는 단계이다.   
   
다음은 마리오게임의 루프 의사코드이다.
```cpp
//마리오게임의 루프 의사코드

void Game::RunLoop()
{
    while (!mShouldQuit) {
        // 입력처리
        JoystickData j = GetJoystickData();

        // 게임 세계 갱신
        UpdatePlayerPosition(j);
        for (PiranhaPlant& pp : mPiranhaPlant) {
            if (pp.Collides(player))
                            뻐끔플라워와 충돌하는 마리오 처리
            else
                pp.Update(); //뻐끔플라워는 프로그램이 제어하는 완전한 AI이므로 이들 또한 로직을 갱신
	}
	// 마리오가 먹는 코인 처리
	// ...

	// 출력 생성
	RenderGraphics();
	RenderAudio();
    }
}
```
   
게임 클래스 골격 구현하기
{: .fs-6 .fw-500 }

**게임 클래스**는 위 게임 루프 코드뿐만아니라 게임을 초기화하고 종료하는 코드를 포함한 클래스이다. 이 클래스는 Game.h파일에 있으며 SDL_Windows 포인터를 참조하므로 메인 SDL 헤더 파일인 SDL/SDL.h를 포함해야 한다.  
```cpp
// Game.h의 게임 클래스(선언부)

class Game
{
public:
    Game();
    // 게임 초기화
    bool Initialize();
    // 게임이 끝나기 전까지 게임 루프를 실행
    void RunLoop();
    // 게임 종료
    void Shutdown();
    
private:
    // 게임 루프를 위한 헬퍼 함수
    void ProcessInput();
    void UpdateGame();
    void GenerateOutput();
    
    // SDL이 생성한 윈도우
    SDL_Window* mWindow;
    bool mIsRunning;
};
```

Game.cpp파일은 다음을 포함한다.
```cpp
// Game.cpp의 게임 클래스(구현부)

// Initialize 함수는 초기화가 성공하면 true를 반환하고 그렇지 않으면 false를 반환한다.
bool Game::Initialize()
{
    // SDL_Init으로 SDL 라이브러리 초기화
    int sdlResult = SDL_Init(SDL_INIT_VIDEO); //SDL_Init의 반환한 값이 0이면 초기화 성공
    if (sdlResult != 0) {
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError()); //C printf 함수와 같은 문법 사용
        return false;
    } 
    
    // SDL_CreateWindow로 SDL 윈도우 생성
    mWindow = SDL_CreateWindow(
        "Game Programming in C++ (Chapter 1)", //윈도우 제목
        100, //윈도우의 Top left x좌표
        100, //윈도우의 Top left y좌표
        1024, //윈도우의 너비
        768, //윈도우의 퐁이
        0 //플래그 (0은 어떠한 플래그도 설정되지 않음을 의미)
        );
    
    if (!mWindow) {
        SDL_Log("Failed to create window: %s", SDL_GetError());
        return false;
    }
    
    return true; //SDL 초기화와 윈도우 생성이 성공하면 Game::Initialize는  true를 반환한다.
} 

// RunLoop 함수는 mIsRunning이 false가 될 때까지 게임 루프를 반복해서 실행한다.
// 각 단계에 대한 3가지 헬퍼 함수가 있으므로 루프 내에서 이 헬퍼 함수만 호출한다.
void Game::RunLoop()
{
    while (mIsRunning) {
        ProcessInput();
        UpdateGame();
        GenerateOutput();
    }
}

// Shutdown 함수는 Initialize 함수와는 반대 역할을 수행한다.
void Game::Shutdown()
{
    // SDL_DestroyWindow로  SDL_Window 객체 해제
    SDL_DestroyWindow(mWindow);
    // SDL_Quit로 SDL을 닫는다
    SDL_Quit();
}
```

C++ 프로그램의 진입점인 main.cpp파일은 다음과 같다.
```cpp
// main

int main(int argc, char** argv)
{
    Game game;
    // 게임 시작
    bool success = game.Initialize();
    // 게임이 초기화 되면 게임 루프에 진입
    if (success)
        game.RunLoop();
    // 루프가 끝나면 게임 종료
    game.Shutdown();
    return 0;
}
```

