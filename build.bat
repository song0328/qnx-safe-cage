@echo off
rem Build the governor and every controller for QNX 8.0 x86_64 (run from any folder).
rem Needs QNX SDP 8.0: set QNX_SDP_ENV to your qnxsdp-env.bat if it is not D:\qnx\qnx800.
if "%QNX_SDP_ENV%"=="" set QNX_SDP_ENV=D:\qnx\qnx800\qnxsdp-env.bat
call "%QNX_SDP_ENV%" > nul
@echo off
set ROOT=%~dp0
set OUT=%ROOT%build\x86_64
if not exist "%OUT%" mkdir "%OUT%"
set CC=qcc -Vgcc_ntox86_64 -O2 -Wall -I"%ROOT%qnx"
%CC% -o "%OUT%\governor" "%ROOT%qnx\governor.c" "%ROOT%qnx\resmgr.c" -lm || (echo BUILD FAIL governor & exit /b 1)
echo built governor
for %%f in ("%ROOT%controllers\*.c") do (
  %CC% -o "%OUT%\%%~nf" "%ROOT%qnx\ctrl_main.c" "%%f" -lm || (echo BUILD FAIL %%~nf & exit /b 1)
  echo built %%~nf
)
