@echo off
chcp 65001 >nul
setlocal
set "S288_EXE="
set "S288_ARGS="
rem Explicit selection is authoritative; do not silently use another environment.
if defined S288_PYTHON goto explicit_python
if exist "%~dp0.venv\Scripts\python.exe" goto local_python
where /q py.exe
if not errorlevel 1 goto windows_python
where /q python.exe
if not errorlevel 1 goto path_python
echo Python was not found. Install Python 3.10+ or set S288_PYTHON to python.exe. 1>&2
exit /b 1

:explicit_python
if not exist "%S288_PYTHON%" goto missing_explicit_python
set "S288_EXE=%S288_PYTHON%"
goto check_python

:local_python
set "S288_EXE=%~dp0.venv\Scripts\python.exe"
goto check_python

:windows_python
set "S288_EXE=py.exe"
set "S288_ARGS=-3"
goto check_python

:path_python
set "S288_EXE=python.exe"

:check_python
"%S288_EXE%" %S288_ARGS% -B -X utf8 -c "import sys; sys.version_info >= (3, 10) or sys.exit('Python 3.10 or newer is required'); import tkinter, PIL, serial"
if errorlevel 1 goto unavailable_python
"%S288_EXE%" %S288_ARGS% -B -X utf8 "%~dp0app.py" %*
exit /b %ERRORLEVEL%

:missing_explicit_python
echo S288_PYTHON does not exist: "%S288_PYTHON%" 1>&2
exit /b 1

:unavailable_python
echo Python or basic dependencies are unavailable in the selected environment. 1>&2
echo Run in the selected environment: 1>&2
echo "%S288_EXE%" %S288_ARGS% -m pip install -r "%~dp0requirements.txt" 1>&2
echo See README.md if Tk is unavailable. 1>&2
exit /b 1
