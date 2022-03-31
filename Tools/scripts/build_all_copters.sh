#!/bin/bash
# Build boarss tested with UAVLAS devices
# Yury Kapacheuski 2021

# set -e
# set -x

# export BUILDROOT="/tmp/all.build"
# rm -rf $BUILDROOT

#Tested Boards
BOARDS="Pixhawk1 Pixhawk4 PH4-mini CUAV-X7 CubeOrange"
# All Boards
#BOARDS="CUAV-Nora CUAV-X7 CUAV-X7-bdshot CUAV_GPS CUAVv5 CUAVv5Nano CubeBlack CubeBlack+ CubeBlack-periph CubeGreen-solo CubeOrange CubeOrange-bdshot CubeOrange-joey CubeOrange-periph CubePurple CubeSolo CubeYellow CubeYellow-bdshot dark disco DrotekP3Pro Durandal Durandal-bdshot edge erleboard erlebrain2 f103-ADSB f103-Airspeed f103-GPS f103-HWESC f103-QiotekPeriph f103-RangeFinder f103-Trigger f303-GPS f303-HWESC f303-M10025 f303-M10070 f303-MatekGPS f303-Universal F35Lightning f405-MatekGPS F4BY FlywooF745 fmuv2 fmuv3 fmuv3-bdshot fmuv5 fmuv5-bdshot FreeflyRTK G4-ESC HitecMosaic HolybroGPS iomcu KakuteF4 KakuteF4Mini KakuteF7 KakuteF7-bdshot KakuteF7Mini linux luminousbee4 luminousbee5 MambaF405v2 MatekF405 MatekF405-bdshot MatekF405-CAN MatekF405-STD MatekF405-Wing MatekF765-Wing MatekH743 MatekH743-bdshot MatekH743-periph MazzyStarDrone mindpx-v2 mini-pix mRoControlZeroClassic mRoControlZeroF7 mRoControlZeroH7 mRoControlZeroH7-bdshot mRoControlZeroOEMH7 mRoNexus mRoPixracerPro mRoPixracerPro-bdshot mRoX21 mRoX21-777 navigator navio navio2 NucleoH743 ocpoc_zynq omnibusf4 omnibusf4pro omnibusf4pro-bdshot omnibusf4pro-one omnibusf4v6 OMNIBUSF7V2 OMNIBUSF7V2-bdshot OmnibusNanoV6 OmnibusNanoV6-bdshot PH4-mini Pix32v5 Pixhawk1 Pixhawk1-1M Pixhawk1-1M-bdshot Pixhawk4 Pixhawk4-bdshot Pixracer Pixracer-bdshot Pixracer-periph pocket pxf pxfmini QioTekZealotF427 R9Pilot revo-mini revo-mini-bdshot revo-mini-i2c revo-mini-i2c-bdshot rst_zynq sitl SITL_arm_linux_gnueabihf sitl_periph_gps SITL_static SITL_x86_64_linux_gnu skyviper-f412-rev1 skyviper-journey skyviper-v2450 sparky2 speedybeef4 SuccexF4 TBS-Colibri-F7 vnav VRBrain-v51 VRBrain-v52 VRBrain-v54 VRCore-v10 VRUBrain-v51 ZubaxGNSS zynq"

mkdir -p build/copters
mkdir -p build/artefacts
git_vesion=`echo \`git describe --tags --dirty --match "ArduCopter*"\`` 

for b in $BOARDS; do
    echo "Starting $b build"
    ./waf configure --board $b
    ./waf clean
    ./waf copter
    mv ${PWD}/build/$b/bin/arducopter.apj ${PWD}/build/copters/$b-$git_vesion.apj
done



tar -zcvf ${PWD}/build/artefacts/uavlas-$git_vesion.tar.gz -C ${PWD}/build/copters .
exit 0
