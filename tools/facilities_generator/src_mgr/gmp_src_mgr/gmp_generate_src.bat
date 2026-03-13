@echo off
setlocal EnableDelayedExpansion

title GMP Source Code Generator
echo =======================================================
echo [GMP] 开始生成/同步源文件 (Source Flatten Mode)
echo =======================================================

:: 检查环境变量
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] 未设置环境变量 GMP_PRO_LOCATION !
    echo [ERROR] 请先将其设置为您的 GMP 核心库根目录。
    pause
    exit /b 1
)

:: 执行源文件同步脚本
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_src_v3.py"

:: 错误捕获
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] 源文件生成失败，已终止流程！
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo 🎉 [SUCCESS] 源文件部署完毕！
echo =======================================================
pause
exit /b 0