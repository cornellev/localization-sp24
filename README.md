# localization-sp24

Utku and Ethan pretend to know what they're doing

# Project Structure

Nodes are written in `src/`.

Eventually, the node structure will look something like this, courtesy of Ethan and Rahul:

![Node structure](./images/structure-transparent-darkmode.png#gh-dark-mode-only)
![Node structure](./images/structure-transparent-lightmode.png#gh-light-mode-only)

# Building

1. Install [microstrain_intertial](https://github.com/LORD-MicroStrain/microstrain_inertial) and [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html). You should be able to use `rosdep`.
2. Just run `catkin_make` in your workspace as usual.

# Running

## LORD Sensor Reader

`lord.launch` can help you verify that the LORD sensor is working correctly. To run it, use `roslaunch localization-sp24 lord.launch`.

### OR: If UTM On Apple Silicon

Run `run_utm_silicon.sh` (you may need to `chmod`).
This may circumvent all troubleshooting issues.

## Sensor Simulator

This node simulates running the LORD sensor from data recorded in a CSV. You can modify the CSV within `test_data` that it reads from within `sim_node.cpp`. You will have to rebuild.

-   To run just the simulator node, use `rosrun localization-sp24 sim_node _test_data_dir:=/path/to/test/data`.
-   To run both the simulator node and robot_localization, just use `roslaunch localization-sp24 sim.launch`. You can change `test_data_dir` from within `sim.launch`.

# Troubleshooting

-   If, upon launching, you get a permission error saying the USB port can't be opened, try giving your user read and write permissions:

```bash
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0
```

-   If the port can't be opened at all, make sure the device is forwarded properly (if using a VM). You may also try to [change the expected device port](https://github.com/LORD-MicroStrain/microstrain_inertial?tab=readme-ov-file#run-instructions) using a params file for the `microstrain_intertial_driver` node.
