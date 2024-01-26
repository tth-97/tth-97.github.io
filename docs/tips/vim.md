---
layout: default
title: vim
parent: Etc
grand_parent: Tips
nav_order: 1
permalink: /docs/tips/etc/vim/
last_modified_date: 2024-01-26 16:59:24
---

vim 
{: .fs-7 .fw-700 }

잘 쓰면 편한 vim.. 잘 써보자!   
<br/>
![vim_cheatsheet](../../../../assets/images/tips/vi_vim_cheat_sheet.gif){: width="90%" height="90%"}
<br/>

----

Commands
{: .fs-6 .fw-700 }  

아래 commands는 normal mode(esc)에서 동작한다.

* Cursor movement
{: .fs-5 .fw-700 .text-yellow-200 }

> **gg**: 문서 첫 줄 시작으로 이동   
> **G**: 문서 끝 줄 시작으로 이동   

<br/>
* Visual mode
{: .fs-5 .fw-700 .text-yellow-200 }  
visually highlight(select) specific text areas and run commands on them)  

> **v**: 비주얼 모드  
> **V**: 비주얼 라인

<br/>
* Edit mode
{: .fs-5 .fw-700 .text-yellow-200 }

> **u**: undo  
> **ctrl+r**: redo

<br/>
* Multi file task
{: .fs-5 .fw-700 .text-yellow-200 }

> **sp [file]**: 상하로 창 분할해서 [file] 열기  
> **vs [file]**: 좌우로 창 분할해서 [file] 열기  
> **ctrl+ww**: 창 전환  
> **ctrl+wk**: 위 창으로 이동  
> **ctrl+wj**: 아래 창으로 이동  
> **ctrl+wh**: 왼쪽 창으로 이동  
> **ctrl+wl**: 오른쪽 창으로 이동  


<br/>

----

Examples
{: .fs-6 .fw-700}

* 전체 선택: gg shift+v+g
* 습관적으로 ctrl+s 누르면 화면이 잠김.. 이럴 땐 ctrl+q


<br/>

-------

참고   
Vim Tips Wiki: [https://vim.fandom.com/wiki/Vim_documentation](https://vim.fandom.com/wiki/Vim_documentation), [http://www.viemu.com/a_vi_vim_graphical_cheat_sheet_tutorial.html](http://www.viemu.com/a_vi_vim_graphical_cheat_sheet_tutorial.html)

