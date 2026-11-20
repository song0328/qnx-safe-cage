@echo off
call D:\qnx\qnx800\qnxsdp-env.bat > nul
qcc -Vgcc_ntox86_64 -O2 -Wall -o D:\qnx-safe-cage\build\abtest D:\qnx-safe-cage\tools\abtest.c && echo built abtest
