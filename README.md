你该如何得到这个工程：
在香橙派中任意打开一个终端，cd /home/orangepi,随后

git clone --recurse-submodules https://github.com/HBUTHUANGPX/livelybot_robot.git

等待结束后，cd livelybot_robot ，catkin build

不出意外的，catkin build编译需要进行两次，这是一个小bug，目前不会去修复，报错仅仅只会在第一次编译时出现。 编译完成后即可。


3.依赖：
(1)Ubuntu 20.04 LTS
(2)ros-noetic
安装方式参考鱼香ros的一键安装,建议安装桌面版:
wget http://fishros.com/install -O fishros && . fishros
(3)dynamic_reconfigure 
sudo apt-get install ros-noetic-dynamic-reconfigure
sudo apt-get install ros-noetic-ddynamic-reconfigure
