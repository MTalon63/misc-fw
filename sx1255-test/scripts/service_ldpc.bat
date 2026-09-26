@echo off
set "PORT=COM6"
echo Starting Modulator Service on %PORT% (LDPC Mode)...
REM Optional: append "    --power 0..100 ^" before the final argument to set
REM initial TX power (100=max/default, 0=min). Omitted = leave hardware unchanged.

python modulator_service.py %PORT% ^
    --rate 250000 ^
    --crate 2 ^
    --ldpc 1 ^
    --rs 0 ^
    --conv 0 ^
    --fecf 0 ^
    --rand 1 ^
    --randpoly 8 ^
    --rrc 1 ^
    --rrc-alpha 0.25 ^
    --rrc-span 0
pause 
