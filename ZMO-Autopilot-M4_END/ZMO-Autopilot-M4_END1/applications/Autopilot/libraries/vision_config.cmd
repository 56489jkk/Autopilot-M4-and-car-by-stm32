@echo off
cd /d %~dp0
title vision_config
::color A4
set /P vision_number=pleas put number[V1.V2.V3]:
echo %vision_number%
if "%vision_number%" == "V1" (
	copy HardwareLib_Board_All\board_interface_v1.c.md HardwareLib_Board\board_interface.c
	copy HardwareLib_Board_All\board_interface_v1.h.md HardwareLib_Board\board_interface.h
	echo "vision_1"
) else if "%vision_number%" == "V2" (
	copy HardwareLib_Board_All\board_interface_v2.c.md HardwareLib_Board\board_interface.c
	copy HardwareLib_Board_All\board_interface_v2.h.md HardwareLib_Board\board_interface.h
	echo "vision_2"
) else if "%vision_number%" == "V3" (
	copy HardwareLib_Board_All\board_interface_v3.c.md HardwareLib_Board\board_interface.c
	copy HardwareLib_Board_All\board_interface_v3.h.md HardwareLib_Board\board_interface.h
	echo "vision_3"
) else (
	echo "error command"
)
pause
