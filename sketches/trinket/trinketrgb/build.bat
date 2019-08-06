@ECHO OFF

REM SET BOARD=SparkFun:avr:promicro:cpu=8MHzatmega32U4
SET BOARD=adafruit:avr:trinket5

PUSHD %~dp0
arduino-cli compile --verbose --fqbn %BOARD%
POPD