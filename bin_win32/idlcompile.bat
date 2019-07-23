
@echo off
@setlocal

if "%1" == "" (
  echo Usage %0 [idl_file]
  goto :end
) 

if not exist rtm_l (
  mkdir rtm_l
)

if exist rtm_l/ (
  omniidl -bpython -Crtm_l %1
) else (
  echo rtm_l is not directory, please remove rtm_l
)

:end
endlocal
echo on