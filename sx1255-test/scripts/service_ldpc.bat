@echo off
set "PORT=COM13"
echo Starting Modulator Service on %PORT% (LDPC Mode)...

python modulator_service.py %PORT% ^
    --rate 400000 ^
    --crate 2 ^
    --ldpc 1 ^
    --rs 0 ^
    --conv 0 ^
    --fecf 1 ^
    --rand 1 ^
    --randpoly 17 ^
    --rrc 1 ^
    --rrc-alpha 0.35 ^
    --rrc-span 0
pause 
