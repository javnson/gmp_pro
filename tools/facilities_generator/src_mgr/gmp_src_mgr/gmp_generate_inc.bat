@echo off
setlocal EnableDelayedExpansion

title GMP Header Tree Generator
echo =======================================================
echo [GMP] 开始生成/同步头文件树 (Header Mirror Mode)
echo =======================================================

:: 检查环境变量
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] 未设置环境变量 GMP_PRO_LOCATION !
    echo [ERROR] 请先将其设置为您的 GMP 核心库根目录。
    pause
    exit /b 1
)

:: 执行头文件同步脚本
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_inc_v3.py"

:: 错误捕获
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] 头文件生成失败，已终止流程！
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo 🎉 [SUCCESS] 头文件树部署完毕！
echo =======================================================
pause
exit /b 0