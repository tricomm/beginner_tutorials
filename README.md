# beginner_tutorials 
## introduction
---
* beginner_tutorials是一个ROS package
* 包中有along2 along1 alongpro 三个ROS Node
* 需要手动创建posts.txt,对小车的行走路线进行指示

## Make&Configuration
---
1. 下载安装系统对应的virtual box虚拟机 [链接](https://www.virtualbox.org/wiki/Downloads)
2. 下载autolaborOS镜像 [链接](http://www.autolabor.com.cn/download)
3. 安装git
    ```
    sudo apt-get upgrade
    sudo apt-get install git
    ```
4. 将小车控制代码clone到本地
    ```
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone  https://github.com/tricomm/beginner_tutorials.git
    ```
5. 编译
    ```
    cd ~/catkin_ws
    catkin_make
    ```
6. 链接对应小车WI-FI
7. 环境变量配置
    1. 检查小车及本地IP
    2. 在本地根据IP创建本地环境变量脚本
    ```
    cd ~/catkin_ws
    vim autolocal.bash #autolabor2.5
    vim prolocal.bash #autolaborpro
    ```
    文件内容
    ```
    # 小车IP及端口 端口号默认为11311
    export ROS_MASTER_URI=http://192.168.2.1:11311
    # 本地IP
    export ROS_IP=192.168.2.84 
    ```
8. ssh远程控制小车编辑小车环境变量脚本
    ```
    cd ~/catkin_ws
    vim auto.bash
    ```
    文件内容
    ```
    # 小车IP及端口 端口号默认为11311
    export ROS_MASTER_URI=http://192.168.2.1:11311
    # 小车IP
    export ROS_IP=192.168.2.84 
    ```
## mapping
---
* 见[autolabor](http://www.autolabor.com.cn/)
## posts.txt 编写(文件可自定义名字)及rviz控制
---
* posts.txt位于工作空间如catkin_ws的根目录下
* rviz可视化控制见[autolabor](http://www.autolabor.com.cn/)
* posts.txt的格式为每行为一个节点,节点参数为三个数分别为x坐标,y坐标,方向
* 坐标系是一个右手系食指为x,中指为y,以食指为旋转0度逆时针为正旋转方向,方向取[-180,180]

    例如
    ```
    0 0 0 
    1 0 90
    1 1 180
    0 1 -90
    0 0 0
    ```
* 编写步骤
    1. 使用rviz控制小车
    2. 打开一个新的`终端1`监听rviz发出的目标位置,输入:
    ```
    cd
    source zsybash/autolocal.bash 
    source catkin_ws/devel/setup.bash
    rosrun beginner_tutorials along1
    rostopic echo /move_base_simple/goal
    ```
    3. 在rviz发出想去的节点位置及方向
    4. 若小车成功到达在`终端1`中查看位置信息并记入posts.txt
    5. 方向信息需要自己填写或通过`终端1`中的四元数进行推算,[转换公式](https://tricomm.github.io./2019/02/28/Quaternion/)
## Run
---
* 小车
 1. autolabor2.5
    ```
    cd catkin_ws
    source auto.bash
    source deve/setup.bash
    cd ....
    roslaunch ...
    ```
 2.  autolaborPro
        ```
        cd catkin_ws
        source auto.bash
        source deve/setup.bash
        ```
        点击桌面上的`开始导航`
* 本地
1. autolabor 2.5
    ```
    cd
    source zsybash/autolocal.bash 
    source catkin_ws/devel/setup.bash
    rosrun beginner_tutorials along1
    ```
2. autolabor pro
   
    终端1
    ```
    cd
    source zsybash/autolocal.bash 
    source catkin_ws/devel/setup.bash
    rosrun beginner_tutorials alongpro
    ```
    终端2
    ```
    cd
    source zsybash/autolocal.bash 
    source catkin_ws/devel/setup.bash
    rosparam set posts_fname [posts路径文件名]
    rosrun beginner_tutorials along2
    注:along2改为along3 小车会从路线中部开始跑
    ```