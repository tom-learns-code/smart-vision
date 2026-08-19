@echo off
call "C:\Program Files\visual stdio2022\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl.exe /nologo /std:c11 /O2 /I. /I..\..\code /c solver_host.c ..\..\code\solver.c ..\..\code\hungarian.c ..\..\code\push_bfs.c ..\..\code\avoidance_graph.c ..\..\code\micro_scheduler.c ..\..\code\bomb_solver_adapter.c
if errorlevel 1 exit /b 1
link.exe /nologo /OUT:solver_host_classic5.exe solver_host.obj solver.obj hungarian.obj push_bfs.obj avoidance_graph.obj micro_scheduler.obj bomb_solver_adapter.obj core_bridge.obj bomb_track.obj bomb_platform_win.obj
if errorlevel 1 exit /b 1
cl.exe /nologo /std:c11 /O2 /I. /I..\..\code /c solver_mode_capacity_test.c
if errorlevel 1 exit /b 1
link.exe /nologo /OUT:solver_mode_capacity_test.exe solver_mode_capacity_test.obj solver.obj hungarian.obj push_bfs.obj avoidance_graph.obj micro_scheduler.obj bomb_solver_adapter.obj core_bridge.obj bomb_track.obj bomb_platform_win.obj
exit /b %errorlevel%
