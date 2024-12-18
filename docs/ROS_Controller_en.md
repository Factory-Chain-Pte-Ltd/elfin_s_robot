Elfin Robot-ROS Controller
======

### ROS-Controller instructions for use
Use this package in ROS control at the same time you can use the tutor or page control, including zero force tutor function, at this time only allows one side of the controller to control the robot, so when the use of non-ROS control, please turn off the ROS controller, the specific open/close method is as follows:

####  How to close
1. Use elfin_gui interface to close it.

Click the CloseROSControl button in the interface to close it, and the color of the button will switch to green after closing.
<p align="center">
  <img src="images/close_control.png" />
  <br>
Close ROS-Controller
</p>

2. Use service /elfin_ros_control/close_ros_control to close it
```sh
rosservice call /elfin_ros_control/close_ros_control "data: true"
```
When close successful you will see the following return:

```
success: True
message: "Success to stop elflin ros controller"
```

#### How to open
Click the OpenROSControl button in the interface to open it, and the color of the button will switch to green after opening.
<p align="center">
  <img src="images/open_control.png" />
  <br>
  Open ROS-Controller
</p>

2. Use serivce /elfin_ros_control/start_ros_control to open it
```sh
rosservice call /elfin_ros_control/open_ros_control "data: true"
```
When open successful you will see the following return:

```
success: True
message: "Success to start elflin ros controller"
```

### Cautions

1. If you use ROS to control the robot arm after closing the ROS-Controller, you will receive the following errors:
<p align="center">
  <img src="images/controller_error.png" />
  <br>
    ROS-Controller not running
</p>

2. If you turn on Zero Force Teaching without turning off the ROS-Controller or use non-ROS to control the arm movement, the ROS-Controller will move the arm to the initial position at the end of the movement, and there will be abnormal movements (jerks or wobbles) during the movement, and the arm will eventually move to the initial position and come to a standstill.