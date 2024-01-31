# localization-sp24

Utku and Ethan pretend to know what they're doing

# Project Structure

Eventually, the node structure will look something like this, courtesy of Ethan and Rahul:

![Node structure](./images/structure-transparent-darkmode.png#gh-dark-mode-only)
![Node structure](./images/structure-transparent-lightmode.png#gh-light-mode-only)

# Progress Tracker

- [x] Research using our existing RTK base station with the LORD IMU
- [ ] First time setup - Reach M+ and Reach RS+

# Usage

## Building

Just run `catkin_make` in your workspace as usual. Nothing interesting here right now.

## Running

1. Make sure you have the [microstrain_intertial](https://github.com/LORD-MicroStrain/microstrain_inertial) package installed.
2. Run `roslaunch localization-sp24 lord.launch` to launch LORD sensor node

## Troubleshooting

- If, upon launching, you get a permission error saying the USB port can't be opened, try giving your user read and write permissions:

```bash
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0
```

- If the port can't be opened at all, make sure the device is forwarded properly (if using a VM). You may also try to [change the expected device port](https://github.com/LORD-MicroStrain/microstrain_inertial?tab=readme-ov-file#run-instructions) using a params file for the `microstrain_intertial_driver` node.
