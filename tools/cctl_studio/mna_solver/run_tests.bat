@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.

set "SOLVER_DIR=%~dp0"
set "GMP_ROOT=%SOLVER_DIR%..\..\.."
for %%I in ("%GMP_ROOT%") do set "GMP_ROOT=%%~fI"
set "GMP_PRO_LOCATION=%GMP_ROOT%"
call "%GMP_ROOT%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 exit /b %ERRORLEVEL%
set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"

set "VALIDATION_TAG=%RANDOM%_%RANDOM%"
set "SIM_CSV=%TEMP%\gmp_mna_sim_%VALIDATION_TAG%.csv"
set "FREQ_CSV=%TEMP%\gmp_mna_freq_%VALIDATION_TAG%.csv"
set "BUCK_CSV=%TEMP%\gmp_mna_buck_%VALIDATION_TAG%.csv"
set "BOOST_CSV=%TEMP%\gmp_mna_boost_%VALIDATION_TAG%.csv"
set "BUCK_JSON=%TEMP%\gmp_mna_buck_%VALIDATION_TAG%.json"
set "BOOST_JSON=%TEMP%\gmp_mna_boost_%VALIDATION_TAG%.json"
set "FSBB_JSON=%TEMP%\gmp_mna_fsbb_%VALIDATION_TAG%.json"

echo [1/12] Compiling Python sources...
python -m py_compile "%SOLVER_DIR%mna_solver.py"
if errorlevel 1 goto :failed
python -m py_compile "%SOLVER_DIR%switched_solver.py"
if errorlevel 1 goto :failed
python -m py_compile "%SOLVER_DIR%circuit_data.py"
if errorlevel 1 goto :failed
python -m py_compile "%SOLVER_DIR%cpp_codegen.py"
if errorlevel 1 goto :failed

echo [2/12] Running unit and supplied-netlist tests...
python -m unittest discover -s "%SOLVER_DIR%tests" -v
if errorlevel 1 goto :failed

echo [3/12] Checking discrete-equation generation...
python "%SOLVER_DIR%mna_solver.py" discretize "%SOLVER_DIR%tb\example6.cir" --dt 1N >nul
if errorlevel 1 goto :failed

echo [4/12] Checking time-domain CSV generation...
python "%SOLVER_DIR%mna_solver.py" simulate "%SOLVER_DIR%tb\example6.cir" --dt 1N --duration 100N --input Vin=1 --output "%SIM_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%SIM_CSV%" goto :missing_output

echo [5/12] Checking frequency-response CSV generation...
python "%SOLVER_DIR%mna_solver.py" frequency "%SOLVER_DIR%tb\example6.cir" --start 10 --stop 1MEG --points 5 --output "%FREQ_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%FREQ_CSV%" goto :missing_output

echo [6/12] Checking Buck and Boost topology generation...
python "%SOLVER_DIR%switched_solver.py" analyze "%SOLVER_DIR%tb_buck\buck.CIR" --dt 1N >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%switched_solver.py" analyze "%SOLVER_DIR%tb_boost\BOOST.CIR" --dt 1N >nul
if errorlevel 1 goto :failed

echo [7/12] Checking 10 kHz Buck and Boost piecewise simulation...
python "%SOLVER_DIR%switched_solver.py" simulate "%SOLVER_DIR%tb_buck\buck.CIR" --dt 1N --duration 60U --pwm-frequency 10K --duty 0.5 --output "%BUCK_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%BUCK_CSV%" goto :missing_output
python "%SOLVER_DIR%switched_solver.py" simulate "%SOLVER_DIR%tb_boost\BOOST.CIR" --dt 1N --duration 60U --pwm-frequency 10K --duty 0.5 --output "%BOOST_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%BOOST_CSV%" goto :missing_output

echo [8/12] Exporting portable Buck, Boost, and FSBB data...
python "%SOLVER_DIR%circuit_data.py" export "%SOLVER_DIR%tb_buck\buck.CIR" "%BUCK_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%BUCK_JSON%" goto :missing_output
python "%SOLVER_DIR%circuit_data.py" simulate "%BUCK_JSON%" --duration 1M --pwm-frequency 10K --duty 0.5 >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%circuit_data.py" export "%SOLVER_DIR%tb_boost\BOOST.CIR" "%BOOST_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%BOOST_JSON%" goto :missing_output
python "%SOLVER_DIR%circuit_data.py" simulate "%BOOST_JSON%" --duration 1M --pwm-frequency 10K --duty 0.5 >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%circuit_data.py" export "%SOLVER_DIR%tb_fsbb\FSBB.CIR" "%FSBB_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%FSBB_JSON%" goto :missing_output

echo [9/12] Regenerating Buck, Boost, and FSBB calculation classes...
call "%SOLVER_DIR%tb_buck\generate_code.bat" buck.CIR --no-pause
if errorlevel 1 goto :failed
call "%SOLVER_DIR%tb_boost\generate_code.bat" BOOST.CIR --no-pause
if errorlevel 1 goto :failed
call "%SOLVER_DIR%tb_fsbb\generate_code.bat" --no-pause
if errorlevel 1 goto :failed

echo [10/12] Compiling and running the Buck Eigen C++ testbench...
call "%SOLVER_DIR%tb_buck\build_testbench.bat" --no-pause
if errorlevel 1 goto :failed

echo [11/12] Compiling and running the Boost Eigen C++ testbench...
call "%SOLVER_DIR%tb_boost\build_testbench.bat" --no-pause
if errorlevel 1 goto :failed

echo [12/12] Compiling and running the handwritten FSBB Eigen C++ testbench...
call "%SOLVER_DIR%tb_fsbb\build_testbench.bat" --no-pause
if errorlevel 1 goto :failed

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
if exist "%BOOST_CSV%" del /q "%BOOST_CSV%" >nul 2>nul
if exist "%BUCK_JSON%" del /q "%BUCK_JSON%" >nul 2>nul
if exist "%BOOST_JSON%" del /q "%BOOST_JSON%" >nul 2>nul
if exist "%FSBB_JSON%" del /q "%FSBB_JSON%" >nul 2>nul
exit /b 0
