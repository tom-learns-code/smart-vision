@echo off
call "C:\Program Files\visual stdio2022\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /O2 /utf-8 /W4 /D_CRT_SECURE_NO_WARNINGS /I. /I.. /I..\..\code adapter_cli.c core_bridge.c ..\..\code\bomb_solver_adapter.c ..\bomb_track.c ..\bomb_platform_win.c /Fe:adapter_cli.exe
