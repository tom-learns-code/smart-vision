@echo off
call "C:\Program Files\visual stdio2022\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /utf-8 /O2 /W4 /D_CRT_SECURE_NO_WARNINGS bomb_cli.c bomb_track.c bomb_platform_win.c /Fe:bomb_cli.exe
