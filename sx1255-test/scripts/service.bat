@echo off
set "PORT=COM6"
echo Starting Modulator Service on %PORT%...
REM Optional: append "    --power 0..100 ^" before the final argument to set
REM initial TX power (100=max/default, 0=min). Omitted = leave hardware unchanged.

python modulator_service.py %PORT% ^
    --rate 300000 ^
    --crate 4 ^
    --rs 1 ^
    --inter 4 ^
    --conv 1 ^
    --fecf 0 ^
    --rand 1 ^
    --randpoly 17 ^
    --rrc 1 ^
    --rrc-alpha 0.25 ^
    --rrc-span 0
pauseb