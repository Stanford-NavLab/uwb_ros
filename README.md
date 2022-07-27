# UWB ROS Repository

This is a ROS metapackage containing packages for calibrating and interfacing with an EVB1000 running the associated compatible [UWB firmware](https://github.com/Stanford-NavLab/uwb-firmware).

## UWB Interface Node
The `uwb_interface` package contains a ROS node that interfaces with the UWBs and publishes the UWB message data onto rostopics for every UWB pair combination.

More details can be found on in the `uwb_interface` package's [README](src/uwb_interface/README.md).

## UWB Calibration
The `uwb_calibration` package contains ROS nodes to calibrate the UWBs' antena delay using published truth ranges from either a constant publisher or from a motion capture system.

More details can be found on in the `uwb_calibration` package's [README](src/uwb_calibration/README.md).


## UWB ROS Usage
To use this ROS metapackage, you should clone it inside of the `src/` directory of a ROS catkin workspace.

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Stanford-NavLab/uwb_ros.git
cd ..
catkin_make
source devel/setup.bash
# run ROS nodes from UWB ROS metapackage!
```
