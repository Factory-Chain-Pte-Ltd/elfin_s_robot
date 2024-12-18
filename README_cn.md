Elfin Robot
======

If you don't speak chinese, please [click here](./README_english.md)

<p align="center">
  <img src="docs/images/elfin.png" />
</p>


本文件夹中包含了多个为Elfin机器人提供ROS支持的软件包。推荐的运行环境为 Ubuntu 18.04 + ROS Melodic, 其他环境下的运行情况没有测试过。

### 安装软件包

#### Ubuntu 18.04 + ROS Melodic

**安装一些重要的依赖包**
```sh
$ ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
```
**安装和升级MoveIt!,** 注意因为MoveIt!最新版进行了很多的优化，如果你已经安装了MoveIt!, 也请一定按照以下方法升级到最新版。

安装/升级MoveIt!：
```sh
$ sudo apt-get update
$ sudo apt-get install ros-melodic-moveit-*
```

安装 trac_ik 插件包
```sh
sudo apt-get install ros-melodic-trac-ik
```

**安装本软件包**

首先创建catkin工作空间 ([教程](http://wiki.ros.org/catkin/Tutorials))。 然后将本文件夹克隆到src/目录下，之后用catkin_make来编译。  
假设你的工作空间是~/catkin_ws，你需要运行的命令如下：
```sh
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/hans-robot/elfin_robot.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```


**安装本软件包**

首先创建catkin工作空间 ([教程](http://wiki.ros.org/catkin/Tutorials))。 然后将本文件夹克隆到src/目录下，之后用catkin_make来编译。  
假设你的工作空间是~/catkin_ws，你需要运行的命令如下：
```sh
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/hans-robot/elfin_robot.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## <font color=Red>注意事项</font>
请注意，本软件包与基本版不同，需要的控制器版本为6.***以上，且请勿随意修改配置文件中的端口名称；在初次使用本版本前请仔细阅读以下注意事项。

使用注意事项：
1. 在本软件包中通过TCP方式控制机器人，可支持在ROS中开启零力示教和设置安全碰撞功能，在开启零力示教之前请确定机器人已经完成了动力学参数辨识，若在机器人末端增加了负载，请先在界面的TCP设置中正确填入有效负载以及重心或进行负载辨识；在连接ROS时若想开启零力示教请先关闭ROS控制器。
2. 若在连接ROS控制器后，示教器或页面中的机器人运动速度比将由ROS控制器控制，且若连接ROS控制器后想使用示教器或页面控制机器人时，请先关闭ROS控制器。
3. 在连接ROS控制器后，在进行任何不是通过ROS控制的运动指令前必须先关闭ROS控制器再执行该指令，否则会造成机器人运动异常。
4. 如何在ROS中关闭ROS控制器，请参考[docs/ROS_Controller.md](./docs/ROS_Controller.md)
---

### 使用仿真模型

***下面给出的是启动Elfin3的一系列命令。启动Elfin5和Elfin10的方法与之类似，只要在相应的地方替换掉相应的前缀即可。***

用Gazebo仿真请运行：
```sh
$ roslaunch elfin_gazebo elfin3_empty_world.launch
```

运行MoveIt!模块, RViz界面:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch
```
如果你此时不想运行RViz界面，请用以下命令:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch display:=false
```

运行后台程序及Elfin Control Panel界面：
```sh
$ roslaunch elfin_basic_api elfin_basic_api.launch
```

> 关于MoveIt!的使用方法可以参考[docs/moveit_plugin_tutorial.md](docs/moveit_plugin_tutorial.md)  
Tips:  
每次规划路径时，都要设置初始位置为当前位置。

---

### 使用真实的Elfin机器人

***下面给出的是启动Elfin3的一系列命令。启动Elfin5和Elfin10的方法与之类似，只要在相应的地方替换掉相应的前缀即可。***

先把购买机器人时得到的elfin_drivers.yaml放到elfin_robot_bringup/config/文件夹下。

将Elfin通过网线连接到电脑。先通过示教器或Elfin界面查看网络设置中的LAN1网口IP。本软件包默认的IP是192.168.0.10 。假如当前IP地址不是192.168.0.10的话，请对elfin_robot_bringup/config/elfin_drivers.yaml的相应部分进行修改。
```
elfin_robot_ip: 192.168.0.10
```

加载Elfin机器人模型：
```sh
$ roslaunch elfin_robot_bringup elfin3_bringup.launch
```
启动Elfin硬件
```sh
$ roslaunch elfin_robot_bringup elfin_ros_control.launch
```

运行MoveIt!模块, RViz界面:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch
```
如果你此时不想运行RViz界面，请用以下命令:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch display:=false
```

运行后台程序及Elfin Control Panel界面：
```sh
$ roslaunch elfin_basic_api elfin_basic_api.launch
```

用Elfin Control Panel界面给Elfin使能指令，如果此时没有报错，直接按下"Servo On"即可使能。如果报错，需先按"Clear Fault"清错后再按下"Servo On"使能。

关于MoveIt!的使用方法可以参考[docs/moveit_plugin_tutorial.md](docs/moveit_plugin_tutorial.md)  
Tips:  
每次规划路径时，都要设置初始位置为当前位置。

在关闭机械臂电源前，需先按下Elfin Control Panel界面的"Servo Off"给Elfin去使能。

更多关于API的信息请看[docs/API_description.md](docs/API_description.md)