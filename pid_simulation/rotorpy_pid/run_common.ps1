$ErrorActionPreference = "Stop"

$ProjectRoot = "D:\DAS\drone\work\cyt4bb7 case01\Seekfree_CYT4BB_Opensource_Library"
$RotorPyRoot = "D:\Aa-app\rotorpy-main"

Set-Location $ProjectRoot
python ".\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py" --common --duration 8 --rotorpy-root $RotorPyRoot
