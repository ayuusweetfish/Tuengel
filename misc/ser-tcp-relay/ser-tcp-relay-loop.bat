@echo off
:loop
%~dp0\ser-tcp-relay.exe COM3
timeout 1
goto loop