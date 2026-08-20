@echo off
setlocal EnableExtensions

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    set "TEST_RESULT=1"
    goto :failed_with_result
)
set "SOLVER_DIR=%GMP_PRO_LOCATION%\tools\cctl_studio\mna_solver"
if not exist "%SOLVER_DIR%\mna_solver.py" (
    echo [ERROR] MNA solver was not found under GMP_PRO_LOCATION:
    echo         %SOLVER_DIR%
    set "TEST_RESULT=1"
    goto :failed_with_result
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed

set "VALIDATION_TAG=%RANDOM%_%RANDOM%"
set "BUCK_CSV=%TEMP%\gmp_mna_buck_%VALIDATION_TAG%.csv"
set "BOOST_CSV=%TEMP%\gmp_mna_boost_%VALIDATION_TAG%.csv"
set "BUCK_JSON=%TEMP%\gmp_mna_buck_%VALIDATION_TAG%.json"
set "BOOST_JSON=%TEMP%\gmp_mna_boost_%VALIDATION_TAG%.json"
set "FSBB_JSON=%TEMP%\gmp_mna_fsbb_%VALIDATION_TAG%.json"
set "SINV_JSON=%TEMP%\gmp_mna_sinv_%VALIDATION_TAG%.json"
set "RECTIFIER_JSON=%TEMP%\gmp_mna_rectifier_%VALIDATION_TAG%.json"
set "INV_JSON=%TEMP%\gmp_mna_inv_%VALIDATION_TAG%.json"
set "BUCK_NPC_JSON=%TEMP%\gmp_mna_buck_npc_%VALIDATION_TAG%.json"

echo [1/12] Compiling Python sources...
python -m py_compile "%SOLVER_DIR%\mna_solver.py" "%SOLVER_DIR%\switched_solver.py" "%SOLVER_DIR%\circuit_data.py" "%SOLVER_DIR%\cpp_codegen.py"
if errorlevel 1 goto :failed

echo [2/12] Running Python unit tests...
python -m unittest discover -s "%SOLVER_DIR%\tests" -v
if errorlevel 1 goto :failed

echo [3/12] Running the basic-netlist acceptance suite...
call "%SOLVER_DIR%\tb\basic\run_tests.bat" --no-pause
if errorlevel 1 goto :failed

echo [4/12] Checking Buck and Boost switched simulation CLIs...
python "%SOLVER_DIR%\switched_solver.py" analyze "%SOLVER_DIR%\tb\buck\buck.CIR" --dt 1N >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%\switched_solver.py" analyze "%SOLVER_DIR%\tb\boost\BOOST.CIR" --dt 1N >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%\switched_solver.py" simulate "%SOLVER_DIR%\tb\buck\buck.CIR" --dt 1N --duration 60U --pwm-frequency 10K --duty 0.5 --output "%BUCK_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%BUCK_CSV%" goto :missing_output
python "%SOLVER_DIR%\switched_solver.py" simulate "%SOLVER_DIR%\tb\boost\BOOST.CIR" --dt 1N --duration 60U --pwm-frequency 10K --duty 0.5 --output "%BOOST_CSV%" >nul
if errorlevel 1 goto :failed
if not exist "%BOOST_CSV%" goto :missing_output

echo [5/12] Checking portable Buck, Boost, FSBB, SINV, rectifier, INV, and NPC Buck data export...
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\buck\buck.CIR" "%BUCK_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%BUCK_JSON%" goto :missing_output
python "%SOLVER_DIR%\circuit_data.py" simulate "%BUCK_JSON%" --duration 1M --pwm-frequency 10K --duty 0.5 >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\boost\BOOST.CIR" "%BOOST_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%BOOST_JSON%" goto :missing_output
python "%SOLVER_DIR%\circuit_data.py" simulate "%BOOST_JSON%" --duration 1M --pwm-frequency 10K --duty 0.5 >nul
if errorlevel 1 goto :failed
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\fsbb\FSBB.CIR" "%FSBB_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%FSBB_JSON%" goto :missing_output
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\sinv\SINV.CIR" "%SINV_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%SINV_JSON%" goto :missing_output
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\rectifier\RECTIFIER.CIR" "%RECTIFIER_JSON%" --normal-dt 1U --short-dt 10N >nul
if errorlevel 1 goto :failed
if not exist "%RECTIFIER_JSON%" goto :missing_output
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\inv\INV.CIR" "%INV_JSON%" --normal-dt 100N --short-dt 1N >nul
if errorlevel 1 goto :failed
if not exist "%INV_JSON%" goto :missing_output
python "%SOLVER_DIR%\circuit_data.py" export "%SOLVER_DIR%\tb\buck_npc\BUCK_NPC.CIR" "%BUCK_NPC_JSON%" --normal-dt 1N --short-dt 100P >nul
if errorlevel 1 goto :failed
if not exist "%BUCK_NPC_JSON%" goto :missing_output

echo [6/12] Generating, compiling, and running the Buck C++ test...
call "%SOLVER_DIR%\tb\buck\build_test.bat" --no-pause
if errorlevel 1 goto :failed

echo [7/12] Generating, compiling, and running the Boost C++ test...
call "%SOLVER_DIR%\tb\boost\build_test.bat" --no-pause
if errorlevel 1 goto :failed

echo [8/12] Generating, compiling, and running the FSBB C++ test...
call "%SOLVER_DIR%\tb\fsbb\build_test.bat" --no-pause
if errorlevel 1 goto :failed

echo [9/12] Generating, compiling, and running the single-phase inverter C++ test...
call "%SOLVER_DIR%\tb\sinv\build_test.bat" --no-pause
if errorlevel 1 goto :failed

echo [10/12] Generating, compiling, and running the controlled-precharge rectifier C++ test...
call "%SOLVER_DIR%\tb\rectifier\build_test.bat" --no-pause
if errorlevel 1 goto :failed

echo [11/12] Generating, compiling, and running the three-phase inverter C++ test...
call "%SOLVER_DIR%\tb\inv\build_test.bat" --no-pause
if errorlevel 1 goto :failed

echo [12/12] Generating, compiling, and running the NPC three-level Buck C++ test...
call "%SOLVER_DIR%\tb\buck_npc\build_test.bat" --no-pause
if errorlevel 1 goto :failed

call :cleanup
echo.
echo All MNA solver tests passed.
if "%NO_PAUSE%"=="0" pause
exit /b 0

:missing_output
echo Expected validation output was not created.
goto :failed

:failed
set "TEST_RESULT=%ERRORLEVEL%"
if "%TEST_RESULT%"=="0" set "TEST_RESULT=1"
:failed_with_result
call :cleanup
echo.
echo MNA solver validation failed with exit code %TEST_RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %TEST_RESULT%

:cleanup
if exist "%BUCK_CSV%" del /q "%BUCK_CSV%" >nul 2>nul
if exist "%BOOST_CSV%" del /q "%BOOST_CSV%" >nul 2>nul
if exist "%BUCK_JSON%" del /q "%BUCK_JSON%" >nul 2>nul
if exist "%BOOST_JSON%" del /q "%BOOST_JSON%" >nul 2>nul
if exist "%FSBB_JSON%" del /q "%FSBB_JSON%" >nul 2>nul
if exist "%SINV_JSON%" del /q "%SINV_JSON%" >nul 2>nul
if exist "%RECTIFIER_JSON%" del /q "%RECTIFIER_JSON%" >nul 2>nul
if exist "%INV_JSON%" del /q "%INV_JSON%" >nul 2>nul
if exist "%BUCK_NPC_JSON%" del /q "%BUCK_NPC_JSON%" >nul 2>nul
exit /b 0
