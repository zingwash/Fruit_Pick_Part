@echo off
setlocal

set "PROJECT_ROOT=%~dp0"
set "DESKTOP_PROJECT=%PROJECT_ROOT%TeachPendant\TeachPendant.csproj"
set "DESKTOP_EXE=%PROJECT_ROOT%TeachPendant\bin\Release\net8.0-windows\TeachPendant.exe"

if not exist "%DESKTOP_EXE%" (
    echo TeachPendant.exe was not found. Building the desktop application...
    dotnet build "%DESKTOP_PROJECT%" -c Release
    if errorlevel 1 (
        echo.
        echo Build failed. Press any key to close this window.
        pause >nul
        exit /b 1
    )
)

start "" "%DESKTOP_EXE%"
exit /b 0
