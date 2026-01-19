## bipedal-robot

该程序是没有手臂版本。

该程序跑在ubuntu20.04上，使用ROS1-Noetic，需要x86架构电脑，**比较吃cpu**。

该程序使用OCS2和ros-control框架，控制方法是非线性模型预测控制，模型采用质心动力学。

该程序是基于[livelybot_dynamic_control](https://github.com/HighTorque-Robotics/livelybot_dynamic_control)、[hunter_bipedal_control](https://bridgedp.github.io/hunter_bipedal_control)以及[legged_control](https://github.com/qiayuanl/legged_control)上进行一些改进，主要是对**约束**进行了修改。

## 学习
1. 理论框架

该程序的数学原理框架可看[基于NMPC和WBC的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630?fromshare=blogdetail&sharetype=blogdetail&sharerId=136541630&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

2. 程序框架

该程序框架是采用ros-control标准格式，对ros-control不太了解的兄弟可先参考用ros-control控制达妙电机的例程[dm-control](https://gitee.com/xauter/dm-control)

## 安装
***安装需要ros基础，这里使用catkin build编译，而不是catkin_make***
### 安装依赖

- [OCS2](https://leggedrobotics.github.io/ocs2/installation.html#prerequisites)

- [ROS1-Noetic](http://wiki.ros.org/noetic)

### 安装 ROS1-Noetic
 这个网上安装教程很多，很简单。
 
### 安装 OCS2
***注意：安装OCS2要保证网络好***

- C++ compiler with C++11 support

- Eigen (v3.3)
```shell
sudo apt-get update
sudo apt-get install libeigen3-dev
```

- Boost C++ (v1.71)
```shell
wget https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.bz2
tar --bzip2 -xf boost_1_71_0.tar.bz2
cd boost_1_71_0
./bootstrap.sh
sudo ./b2 install
```
如果解压boost_1_71\_0.tar.bz2失败，可以自己去官网下载boost_1_71_0.tar.bz2,官网链接如下：

[https://www.boost.org/users/history/](https://www.boost.org/users/history/)


- 安装剩余依赖
```shell
sudo apt install ros-noetic-catkin
sudo apt install libglpk-dev libglfw3-dev
sudo apt install ros-noetic-pybind11-catkin
sudo apt install python3-catkin-tools
sudo apt install doxygen doxygen-latex
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
sudo apt-get install ros-noetic-rqt-multiplot
sudo apt install ros-noetic-grid-map-msgs
sudo apt install ros-noetic-octomap-msgs
sudo apt install libreadline-dev
sudo apt install libcgal-dev
sudo apt update && sudo apt install checkinstall
sudo apt-get install libserialport0 libserialport-dev
sudo apt install ros-noetic-serial
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-joy-teleop
```

- 安装相关库
1. 针对ocs2单独创建一个工作空间
```shell
mkdir -p ~/ocs2_ws/src
cd ~/ocs2_ws/src
```
2. 下载功能包***（先不编译）***

在~/ocs2_ws/src目录下，打开终端，输入：
```shell
 git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
 git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
 git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
 git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01
 git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git
 git clone https://github.com/ANYbotics/grid_map.git
 git clone https://github.com/leggedrobotics/ocs2.git
```
![](./src/docs/catalog_folder.png)

3. 编译Raisim
```shell
cd ~/ocs2_ws/src/raisimLib
mkdir build
cd build
cmake .. 
make -j4 && sudo checkinstall
```

4. ONNX Runtime 依赖
```shell
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
tar xf onnxruntime-linux-x64-1.7.0.tgz
mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib
rsync -a ~/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime
mkdir -p ~/.local/share/onnxruntime/cmake/
rsync -a ~/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/onnxruntime/cmake/
```
然后需要在.bashrc手动设置环境变量，否则在后续安装ocs2时，容易找不到该软件包
打开终端，输入：
```shell
cd
gedit .bashrc
```
然后在最后一行输入：
```shell
export onnxruntime_DIR=~/.local/
export LD_LIBRARY_PATH=~/.local/lib:${LD_LIBRARY_PATH}
```

5. 编译（最关键的步骤来了，**编译需要在ocs2_ws目录下进行**）

首先，优先编译elevation_mapping_cupy

***注意：一定要添加catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo，如下所示***
```shell
cd ~/ocs2_ws/
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build elevation_mapping_cupy
```
没报错的话，编译grid_map
```shell
catkin build grid_map
```
没报错的话，编译hpp-fcl
```shell
catkin build hpp-fcl
```
没报错的话，编译pinocchio
```shell
catkin build pinocchio
```
没报错的话，编译ocs2_robotic_assets
```shell
catkin build ocs2_robotic_assets
```
没报错的话，编译ocs2（这一步编译要等很长时间）
```shell
catkin build ocs2
```

6. 测试

编译完成之后需要在.bashrc手动设置环境变量
打开终端，输入：
```shell
cd
gedit .bashrc
```
然后在.bashrc文件里最后一行输入：
```shell
source ~/ocs2_ws/devel/setup.bash
```
然后在终端输入：
```shell
source .bashrc
```
然后启动ocs2的一个四足例程（首次运行需要等一下，ocs2有个预先计算，等待rviz出现图形）
```shell
roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch
```
<img src="./src/docs/leg_example.png" width="500" height="auto">

OCS2安装成功！！！！

### 安装和编译双足控制程序

首先打开终端，输入：
```shell
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```
然后把gitee上的src文件夹放到catkin_ws目录下，如下所示（不能有其他东西）
![](./src/docs/src.png)

打开终端，输入：
```shell
cd ~/catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
```
然后在.bashrc文件里最后一行输入：
```
source ~/catkin_ws/devel/setup.bash
```
然后打开终端，输入：
```
cd
source .bashrc
```

## 仿真与实机的运行
***注意：一开始运行实机前最好先运行仿真，检查仿真没问题后，再运行实机***
### 仿真，分两种情况，一种是有遥控器，一种是没有遥控器
#### 没有遥控器
1. 运行gazebo仿真并且载入控制器:

```shell
roslaunch legged_controllers one_start_gazebo.launch
```
<img src="./src/docs/sim.png" width="600" height="auto">

2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在gazebo里按住键盘***Ctrl+Shift+R***让机器人站起来。

3. 新建一个终端，发布/reset_estimation话题，重置状态:
```shell
rostopic pub --once /reset_estimation std_msgs/Float32 "data: 0.0" 
```
4. 新建一个终端，发布/cmd_vel话题，**这里速度不能为0**，不然话题无法发布出去:
```bash
rosrun rqt_robot_steering rqt_robot_steering 
```
<img src="./src/docs/vel.png" width="300" height="auto">

5. 新建一个终端，开启控制器：
```shell
rostopic pub --once /load_controller std_msgs/Float32 "data: 0.0" 
```
6. 最后发布/set_walk话题:
```shell
rostopic pub --once /set_walk std_msgs/Float32 "data: 0.0" 
```

#### 有遥控器（能支持ros的遥控器都能直接用）

1. 运行gazebo仿真并且载入控制器:
```shell
roslaunch legged_controllers one_start_gazebo.launch
```
2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在gazebo里按住键盘***Ctrl+Shift+R***让机器人站起来。

后面的步骤就使用遥控器按键，每一个按键对应一个话题，对应关系如下图所示：

<img src="./src/docs/handle.png" width="500" height="auto">

### 实机

1. 给stm32h7开发板烧录下位机程序，上电，此时所有电机应该全亮绿灯

2. 在你的电脑里运行上位机程序
首先检查开发板和陀螺仪的连接
```shell
cd
ls /dev/ttyACM*
```
<img src="./src/docs/dev2.png" width="450" height="auto">

***注意：/dev/ttyACM0是单片机设备，/dev/ttyACM1是IMU设备，两者要各自对应***

然后给用usb连接的开发板和陀螺仪设置权限
```shell
sudo chmod -R 777 /dev/ttyACM*
```
接着运行上位机程序
```shell
roslaunch legged_controllers one_start_real.launch
```
此时rviz中的机器人模型不动，并且姿势和实机一样

2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在现实世界里扶正机器人，机器人此时稳定站立。

后面的步骤和上面仿真一样，另外遥控器连接后打开遥控器开关，它会自动发布cmd\_cel话题，不需要再运行rosrun rqt\_robot\_steering rqt_robot\_steering 




## 参考

### 代码参考

https://github.com/bridgedp/hunter_bipedal_control

https://github.com/HighTorque-Robotics/livelybot_dynamic_control

https://github.com/qiayuanl/legged_control

### 文献参考
[OCS2安装参考1](https://blog.csdn.net/Study__ing_/article/details/140177463?fromshare=blogdetail&sharetype=blogdetail&sharerId=140177463&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[OCS2安装参考2](https://blog.csdn.net/m0_52545777/article/details/140276558?fromshare=blogdetail&sharetype=blogdetail&sharerId=140276558&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[基于NMPC和WBC的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630?fromshare=blogdetail&sharetype=blogdetail&sharerId=136541630&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[OCS2代码解析：Quadrotor]https://zhuanlan.zhihu.com/p/687952253

```
# State Estimation

[1] Flayols T, Del Prete A, Wensing P, et al. Experimental evaluation of simple estimators for humanoid robots[C]//2017 IEEE-RAS 17th International Conference on Humanoid Robotics (Humanoids). IEEE, 2017: 889-895.

[2] Bloesch M, Hutter M, Hoepflinger M A, et al. State estimation for legged robots-consistent fusion of leg kinematics and IMU[J]. Robotics, 2013, 17: 17-24.

# MPC

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the mit cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] Grandia R, Jenelten F, Yang S, et al. Perceptive Locomotion Through Nonlinear Model-Predictive Control[J]. IEEE Transactions on Robotics, 2023.

[5] Sleiman J P, Farshidian F, Minniti M V, et al. A unified mpc framework for whole-body dynamic locomotion and manipulation[J]. IEEE Robotics and Automation Letters, 2021, 6(3): 4688-4695.

# WBC

[6] Bellicoso C D, Gehring C, Hwangbo J, et al. Perception-less terrain adaptation through whole body control and hierarchical optimization[C]//2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids). IEEE, 2016: 558-564.

[7] Kim D, Di Carlo J, Katz B, et al. Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control[J]. arXiv preprint arXiv:1909.06586, 2019.
```
