@echo off
echo Starting Modulator Service on COM8...

python modulator_service.py COM13 ^
    --rate 250000 ^
    --crate 2 ^
    --rs 1 ^
    --inter 4 ^
    --conv 1 ^
    --fecf 0 ^
    --rand 1 ^
    --randpoly 17 ^
    --rrc 1 ^
    --rrc-alpha 0.25 ^
    --rrc-span 0
pause