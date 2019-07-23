
@echo off
@setlocal

if "%1" == "" (
  echo Usage %0 [idl_file]
  goto :end
) 

if not exist rtm_idl (
  mkdir rtm_idl
)

if exist rtm_idl/ (
  omniidl -bpython -Crtm_idl %1
) else (
  echo rtm_idl is not directory, please remove rtm_idl
)

:end
endlocal
echo on