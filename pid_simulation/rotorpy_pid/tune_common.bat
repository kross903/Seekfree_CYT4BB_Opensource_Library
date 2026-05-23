@echo off
setlocal

set "PROJECT_ROOT=D:\DAS\drone\work\cyt4bb7 case01\Seekfree_CYT4BB_Opensource_Library"
set "ROTORPY_ROOT=D:\Aa-app\rotorpy-main"

cd /d "%PROJECT_ROOT%"
python ".\pid_simulation\rotorpy_pid\interactive_rotorpy_pid.py" --duration 5 --rotorpy-root "%ROTORPY_ROOT%"

endlocal
