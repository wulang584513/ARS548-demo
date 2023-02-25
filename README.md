# ARS548-demo

# ReadMe

## 一 运行环境说明
本例程测试环境：硬件为英伟达Xavier，操作系统为Ubuntu18.04，ROS版本为melodic。
雷达与Xavier的eth0网口连接，Xavier设置IP地址为10.13.1.166。雷达与Xavier之间还串接了以太网转换模块和交换机，
其中交换机需要进行VLAN19设置，有关交换机的设置，请参考文档《采用交换机设置VLAN的方法.pdf》

## 二 编译和运行

### 1 编译

(1)终端切换到ars548_demo路径

(2)首次运行需要编译，再次运行不需要。删除旧的编译环境 rm build/ devel/ -fr

(3)编译 catkin_make(代码修改后只需要编译,不需要删除build 和 devel)

### 2 运行

(1)保存环境变量source devel/setup.bash

(2)如果需要每次都可以正常运行ROS节点，需要把环境变量保存到~/.bashrc文件中。
如：在~/.bashrc中添加source ~/Documents/ars548_demo/devel/setup.bash

(3)运行 

可以通过一个命令启动所有节点, 命令:roslaunch ars548_process ars548_process.launch 

本ROS驱动中，所有节点都在ars548_process包中，这个包中包含雷达数据解析节点ars548_process_node、
雷达数据转换节点info_convert_node、雷达输入信号测试节点test_radar_input_node、以及rviz的配置文件ars548_display.rviz

也可以使用rosrun独立运行每个节点。

## 三 代码结构说明

代码包含公用消息包ars548_msg和雷达信号处理包ars548_process

### 1 公用消息包ars548_msg

存储公用消息，存储在ars548_msg中，可以查看包中的msg文件夹下的.msg文件来确认每个消息的定义。

### 2 雷达信号处理包ars548_process

在这个包中，包含两个节点ars548_process_node、info_convert_node和test_radar_input_node。

#### 2.1 ars548_process_node节点
ars548_process_node节点用来解析雷达信号，通过网口接收雷达发送来的UDP数据。经过解析后把信息发布出来；同时
ars548_process_node节点接收车辆状态信息的话题，并将车辆状态通过UDP发送给雷达。

ars548_process_node发布的话题有：
/ars548_process/object_list
/ars548_process/detection_list
/ars548_process/radar_status

每个话题的内容如下：
/ars548_process/object_list
雷达探测的object列表
/ars548_process/detection_list
雷达探测的detection列表
/ars548_process/radar_status
雷达的状态信息

ars548_process_node接收的话题有：
/ars548_process/acc_lateral_cog
/ars548_process/acc_longitudinal_cog
/ars548_process/characteristic_speed
/ars548_process/driving_direction
/ars548_process/steering_angle
/ars548_process/velocity_vehicle
/ars548_process/yaw_rate

每个话题的内容如下：
/ars548_process/acc_lateral_cog
车辆的侧向加速度状态信息
/ars548_process/acc_longitudinal_cog
车辆的纵向加速度状态信息
/ars548_process/characteristic_speed
车辆的特性车速信息
/ars548_process/driving_direction
车辆的行驶方向信息
/ars548_process/steering_angle
车辆的前轴转向角信息
/ars548_process/velocity_vehicle
车辆的车速信息
/ars548_process/yaw_rate
车辆的横摆角速度信息

#### 2.2 info_convert_node节点
info_convert_node节点用来将object和detection列表的数据转换为rviz可以显示的数据格式。
这个节点接收ars548_process_node节点发布/ars548_process/object_list话题/ars548_process/detection_list话题
经过格式转换之后，发布成rviz可以展示的格式。发布的话题包括：
/ars548_process/object_marker
用来显示object的话题
/ars548_process/detection_point_cloud
用来显示detection的话题

ars548_display.rviz是rviz的配置文件。

#### 2.3 test_radar_input_node节点
test_radar_input_node节点用来测试雷达的输入信号，
为了使雷达的工作状态达到最佳，需要给雷达输入车辆的状态信息，在实车上，需要安装相应的传感器来测量车辆状态。
在这个demo中，我们把所有状态信息都设置成默认状态，并发布出去。
test_radar_input_node节点发布的话题被ars548_process_node节点接收，并发送给雷达。
test_radar_input_node节点发布的话题如下：
/ars548_process/acc_lateral_cog
/ars548_process/acc_longitudinal_cog
/ars548_process/characteristic_speed
/ars548_process/driving_direction
/ars548_process/steering_angle
/ars548_process/velocity_vehicle
/ars548_process/yaw_rate


