#! /bin/bash

printf "Press ENTER to confirm you are running this script on Apple Silicon in UTM..."
read
printf "Press ENTER to confirm you are in the Lord IMU workspace..."
read
printf "Click the USB port button in UTM and select the Lord IMU port. Press ENTER when ready..."
read
sudo usermod -a -G dialout $USER
if [[ -z $(ls /dev | grep ACM0) ]]; then
  echo "Port ACM0 not found"
  exit 1
fi
sudo chmod a+rw /dev/ttyACM0
source devel/setup.bash
roslaunch localization-sp24 lord.launch
