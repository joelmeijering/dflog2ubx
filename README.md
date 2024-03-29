# Ardupilot DataFlash binary log file to UBX binary log

!!!IMPORTANT!!!: Only works with a patched ArduPilot, since as of today (March 29, 2024) the sigId field is missing from the dataflash logs. It will likely be fixed soon.

See https://discuss.ardupilot.org/t/ap-gps-ublox-driver-produces-invalid-and-missing-rawh-and-raws-log-messages/115380

- Converts an Ardupilot DataFlash log file (.bin) to a U-blow raw stream.
- It only extracts the RAWX messages.
- RAWX messages are only present when GPS_RAW_DATA is enabled.
- When your DataFlash log is collected with an unpatched ArduPilot version, this code will not work!

# Building

Tested on Ubuntu LTS 22
Packages needed: build-essential, cmake, git

This code relies on an Ardupilot folder being present, and it must been built as described below, since otherwise some Ardupilot headers will be missing. 

## 1. Prepare ardupilot folder

git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
./waf configure --board=sitl
./waf copter

Note down the path to your Ardupilot folder.

## 2. Configure and buld

cd dflog2ubx
mkdir build
cd build
cmake .. -DARDUPILOT_FOLDER=MY_LITTLE_FOLDER
make

Note: Replace MY_LITTLE_FOLDER with the pull path to your ArduPilot folder.

## 3. Run

./dflog2ubx DFLOG_FILENAME.bin

Replace DFLOG_FILENAME with full path to your dflog file (no relative paths work I think).

It will automatically create a .ubx file in the same folder as the .bin file. If a .ubx file is already present, it will be deleted.