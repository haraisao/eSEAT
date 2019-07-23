@set PYTHON_DIR=
@set PYTHON_PATHS=
@FOR /F %%i in ( '%~dp0..\bin\find_python.bat' ) do @(
  @set PYTHON_DIR=%%i
  @set PYTHON_PATHS=%%i;%%i\Scripts
)
@set PYTHONPATH=rtm_idl;%PYTHONPATH%
@set PATH=%~dp0..\bin;%PATH%
