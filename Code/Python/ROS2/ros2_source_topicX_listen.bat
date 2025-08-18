@echo off
REM Check if the argument is provided
if "%1"=="" (
    echo Usage: %0 topic_name
    exit /b 1
)

REM Call the local setup script
call C:\dev\ros2_humble\local_setup.bat

REM Use the provided argument as the topic name
ros2 topic echo /%1