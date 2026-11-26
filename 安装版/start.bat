@echo off
rem 北斗三号终端测试工具 - 启动脚本
setlocal
rem 不生成 __pycache__ 字节码缓存
set PYTHONDONTWRITEBYTECODE=1
set "HERE=%~dp0"
set "BD3=%HERE%bd3_test.py"
if not exist "%BD3%" (
    echo 找不到 bd3_test.py，请把本批处理文件与脚本放在同一目录。
    pause
    exit /b 1
)

rem 1) 优先使用 py 启动器
py -3 -c "import sys" >nul 2>nul
if %errorlevel% equ 0 (
    py -3 "%BD3%"
    goto :done
)

rem 2) 使用 PATH 中的 python，排除应用商店的存根
for /f "delims=" %%P in ('where python 2^>nul') do (
    echo %%P | findstr /i "WindowsApps" >nul
    if errorlevel 1 (
        "%%P" -c "import sys" >nul 2>nul
        if not errorlevel 1 (
            "%%P" "%BD3%"
            goto :done
        )
    )
)

echo 未找到可用的 Python 3，请先安装 Python 3.7 以上版本。
pause
exit /b 1

:done
endlocal