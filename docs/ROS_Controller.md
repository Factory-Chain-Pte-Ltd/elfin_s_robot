Elfin Robot-ROS Controller
======

### ROS-Controller使用说明
使用本软件包在ROS中控制的同时可以使用示教器或页面进行控制,包括零力示教功能,此时只允许一方控制器去控制机器人,所以当使用非ROS控制时请先关闭ROS控制器,具体开启/关闭方法如下:

####  关闭方法
1. 使用elfin_gui界面关闭

点击界面中的CloseROSControl按钮即可关闭,关闭后该按钮颜色切换到绿色
<p align="center">
  <img src="images/close_control.png" />
  <br>
  关闭ROS-Controller
</p>

2. 使用/elfin_ros_control/close_ros_control服务关闭
```sh
rosservice call /elfin_ros_control/close_ros_control "data: true"
```
当关闭成功时将会看到如下返回:

```
success: True
message: "Success to stop elflin ros controller"
```

#### 开启方法
点击界面中的OpenROSControl按钮即可开启,开启后该按钮颜色切换到绿色
<p align="center">
  <img src="images/open_control.png" />
  <br>
  开启ROS-Controller
</p>

2. 使用/elfin_ros_control/start_ros_control服务关闭
```sh
rosservice call /elfin_ros_control/open_ros_control "data: true"
```
当关闭成功时将会看到如下返回:

```
success: True
message: "Success to start elflin ros controller"
```

### 注意事项

1. 在关闭ROS-Controller后,若使用ROS控制机械臂将会收到如下错误
<p align="center">
  <img src="images/controller_error.png" />
  <br>
    ROS-Controller未运行
</p>

2. 若在未关闭ROS-Controller的情况下开启零力示教或使用非ROS控制机械臂运动,在运动结束后ROS控制器将会将机械臂运动到初始位置,且运动过程中会出现异常动作(抖动或摆动),最终会运动到初始位置并静止