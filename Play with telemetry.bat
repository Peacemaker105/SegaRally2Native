@echo off
rem Starts Sega Rally 2 with the motion-rig telemetry streaming to host:port.
rem Currently pointed at VN Motion Dashboard's Forza source: 127.0.0.1:5300.
rem Forza-only format so the dashboard sees clean 324-byte packets (no JSON).
rem Change the address if the dashboard runs on another machine (e.g. 192.168.1.50:5300).
set "SR2_TELEMETRY=127.0.0.1:5300@forza"
start "" "%~dp0SegaRally2.exe"
