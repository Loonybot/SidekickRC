@echo off
if "%1"=="" (
    echo Invoke it like this: "lib-release 0.0.0"
    exit /b)
git tag -a lib-v%1 -m "Sidekick Library %1"
git push origin lib-v%1