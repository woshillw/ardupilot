## build
./waf configure --board CubeTSDV1   

./waf copter

./waf --targets bin/arducopter --upload

## bootloader
./waf configure --board CubeTSDV1  --bootloader

./waf copter
