@echo off
setlocal EnableExtensions

set "SOLVER_DIR=%~dp0"
set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"

set "VALIDATION_TAG=%RANDOM%_%RANDOM%"
set "SIM_CSV=%TEMP%\gmp_mna_sim_%VALIDATION_TAG%.csv"
set "FREQ_CSV=%TEMP%\gmp_mna_freq_%VALIDATION_TAG%.csv"
set "BUCK_CSV=%TEMP%\gmp_mna_buck_%VALIDATION_TAG%.csv"

echo [1/7] Compiling Python sources...
python -m py_compile "%SOLVER_DIR%mna_solver.py"
if errorlevel 1 goto :failed
python -m py_compile "%SOLVER_DIR%switched_solver.py"
if errorlevel 1 goto :failed

echo [2/7] Running unit and supplied-netlist tests...
python -m unittest discover -s "%SOLVER_DIR%tests" -v
if errorlevel 1 goto :failed

echo [3/7] Checking discrete-equation generation...
python "%SOLVER_DIR%mna_solver.py" discretize "%SOLVER_DIR%tb\example6.cir" --dt 1N >nul
if errorlevel 1 goto :failed

echo [4/7] Checking time-domain CSV generation...
python "%SOLVER_DIR%mna_solver.py" simulate "%SOLVER_DIR%tb\example6.cir" --dt 1N --duration 100N --input Vin=1 --output "%SIM_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%SIM_CSV%" goto :missing_output

echo [5/7] Checking frequency-response CSV generation...
python "%SOLVER_DIR%mna_solver.py" frequency "%SOLVER_DIR%tb\example6.cir" --start 10 --stop 1MEG --points 5 --output "%FREQ_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%FREQ_CSV%" goto :missing_output

echo [6/7] Checking switched-topology equation generation...
python "%SOLVER_DIR%switched_solver.py" analyze "%SOLVER_DIR%tb\2_buck.CIR" --dt 1N >nul
if errorlevel 1 goto :failed

echo [7/7] Checking 10 kHz PWM piecewise simulation...
python "%SOLVER_DIR%switched_solver.py" simulate "%SOLVER_DIR%tb\2_buck.CIR" --dt 1N --duration 60U --pwm-frequency 10K --duty 0.5 --output "%BUCK_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%BUCK_CSV%" goto :missing_output

call :cleanup
echo.
echo All MNA solver tests passed.
if "%NO_PAUSE%"=="0" pause
exit /b 0

:missing_output
echo Expected validation CSV was not created.
goto :failed

:failed
set "TEST_RESULT=%ERRORLEVEL%"
if "%TEST_RESULT%"=="0" set "TEST_RESULT=1"
call :cleanup
echo.
echo MNA solver validation failed with exit code %TEST_RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %TEST_RESULT%

:cleanup
if exist "%SIM_CSV%" del /q "%SIM_CSV%" >nul 2>nul
if exist "%FREQ_CSV%" del /q "%FREQ_CSV%" >nul 2>nul
if exist "%BUCK_CSV%" del /q "%BUCK_CSV%" >nul 2>nul
exit /b 0
