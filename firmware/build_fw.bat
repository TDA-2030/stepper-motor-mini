
@echo off

@REM SET mcu=stm32f103
SET mcu=gd32f1x0

md build
cd build
cmake.exe -DCMAKE_BUILD_TYPE=MinSizeRel -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -G "MinGW Makefiles" ../%mcu%

make -j8

cd ..

