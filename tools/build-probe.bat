@echo off
call D:\qnx\qnx800\qnxsdp-env.bat > nul
qcc -Vgcc_ntox86_64 -O2 -Wall -ID:\qnx-safe-cage\qnx -o D:\qnx-safe-cage\build\x86_64\probe D:\qnx-safe-cage\qnx\ctrl_main.c D:\qnx-safe-cage\tools\probe.c && echo built probe
