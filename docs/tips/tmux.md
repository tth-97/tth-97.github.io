---
layout: default
title: tmux
parent: Etc
grand_parent: Tips
nav_order: 1
permalink: /docs/tips/etc/tmux/
---

tmux
{: .fs-7 .fw-700 }

[tmux](https://github.com/tmux/tmux)는 터미널에서 실행되는 하나의 프로그램으로, 여러 다른 터미널 프로그램을 내부에서 실행할 수 있게 해준다. tumx 내부의 각 프로그램은 tmux가 관리하는 자체 터미널을 가지고 있으며, 이 터미널은 tmux가 실행 중인 단일 터미널에서 접근할 수 있다. 이를 mutiplexing이라고 하며, tmux는 terminal multiplexer이다.

tmux와 그 안에서 실행되는 모든 프로그램은 외부 터미널에서 분리(detach)할 수 있고, 나중에 동일한 터미널이나 다른 터미널에 다시 연결(reattach)할 수 있다. datach된 프로그램은 화면에서 분리되지만 **background에서 계속 실행**된다.

tmux의 목적은 다음과 같다.
* tumx 내 원격 서버에서 실행 중인 프로그램을 연결이 끊겨도 보호한다.
* 원격 서버에서 실행 중인 프로그램을 여러 로컬 컴퓨터에서 접근할 수 있도록 한다.
* 여러 프로그램과 쉘을 하나의 터미널에서 함께 작업할 수 있도록 한다.

예를 들어 내가 작업용 컴퓨터의 xterm에서 ssh를 사용하여 원격 서버에 연결하고 여러 프로그램(editor, compiler, shell 등)으로 작업을 수행했다고 하자. 만약 tmux로 xterm을 닫고 집에 간다면, 집에서 동일한 원격 서버에 연결하고 tmux에 접속하여 이전 작업을 이어서 진행할 수 있을 것이다.
또한 ssh 등을 사용하여 원격 접속을 하다 ssh가 중단되면, 그 세션에서 foreground로 실행중인 프로그램도 중단이 된다. 만약 tmux를 사용하여 background에서 프로그램을 실행한다면 SSH 세션이 닫혀도 작업이 중단되지 않고 실행될 수 있을 것이다.

<br/>
---

Installation
{: .fs-6 .fw-700 }

Debian 또는 Ubuntu에서 Binary packages로 설치하는 명령어이다.

```yaml
$ apt install tmux
```

<br/>

---

Basic concepts
{: .fs-6 .fw-700 }

tmux는 모든 state를 하나의 메인 process인 tmux server에 보관한다. 이 서버는 background에서 실행되며, tmux 아넹서 실행 중인 모든 프로그램을 관리하고 output을 추적한다. 사용자가 tmux 명령을 실행하면 tmux server가 자동으로 시작되며, 기본적으로 실행 중인 프로그램이 없을 때 종료된다.

사용자는 client를 시작하며 tmux server에 attach한다. 이 client는 실행되는 터미널을 점유하며, /tmp 디렉토리에 있는 socket file을 통해 server와 통신한다. 각 client는 하나의 터미널에서 실행된다.

 
----

Cost

