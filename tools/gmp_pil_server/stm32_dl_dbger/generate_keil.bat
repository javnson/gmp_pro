@echo off
setlocal

if defined STM32CUBEMX_HOME (
    set "CUBEMX_HOME=%STM32CUBEMX_HOME%"
) else (
    set "CUBEMX_HOME=%ProgramFiles%\STMicroelectronics\STM32Cube\STM32CubeMX"
)

if not exist "%CUBEMX_HOME%\STM32CubeMX.exe" (
    echo [ERROR] STM32CubeMX was not found at "%CUBEMX_HOME%".
    exit /b 1
)

pushd "%~dp0"
"%CUBEMX_HOME%\jre\bin\java.exe" -jar "%CUBEMX_HOME%\STM32CubeMX.exe" -q generate_keil.txt
if errorlevel 1 goto :fail

python patch_keil_project.py
if errorlevel 1 goto :fail

popd
echo Keil project generation completed.
exit /b 0

:fail
popd
echo [ERROR] Keil project generation failed.
exit /b 1
