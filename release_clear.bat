@echo off

for /f "delims=" %%d in ('dir /b /s /ad .github') do (
    if exist "%%d" (
        echo remove: "%%d"
        rd /s /q "%%d"
    )
)

echo.
echo Operation complete.
echo.
pause