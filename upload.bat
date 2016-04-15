@echo off
mode %1 BAUD=38400 PARITY=n DATA=8 RTS=OFF DTR=ON TO=OFF
rem echo U > %1
