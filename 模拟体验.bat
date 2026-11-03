@echo off
chcp 65001 >nul
rem Transfer to the shared launcher without a second CALL expansion of arguments.
"%~dp0启动工具.bat" --demo %*
