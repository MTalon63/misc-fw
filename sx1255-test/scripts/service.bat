@echo off
set "PORT=COM13"
echo Starting Modulator Service on %PORT%...

python modulator_service.py %PORT% ^
    --rate 400000 ^
    --crate 1 ^
    --rs 1 ^
    --inter 4 ^
    --conv 1 ^
    --fecf 0 ^
    --rand 1 ^
    --randpoly 17 ^
    --rrc 1 ^
    --rrc-alpha 0.35 ^
    --rrc-span 0
pause