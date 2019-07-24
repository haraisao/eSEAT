@set PYTHON_DIR=
@set PYTHON_PATHS=
@FOR /F %%i in ( '%~dp0..\bin\find_python.bat' ) do @(
  @set PYTHON_DIR=%%i
  @set PYTHON_PATHS=%%i;%%i\Scripts;%%i\DLLs
)
@set SEAT_ROOT=%~dp0
@set PATH=%PYTHON_PATHS%;%~dp0..\bin;%PATH%
