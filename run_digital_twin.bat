@echo off
rem Mo Digital Twin ABU Robocon 2027 tai http://localhost:8000
cd /d "%~dp0"
start "" http://localhost:8000/
python -m http.server 8000
