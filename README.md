# Ardupilot Gazebo plugin 

这不是一个ROS包，编译命令如下（要source了devel/setup.bash，不然找不到RSOS）：

```
# 1. 创建并进入构建目录
mkdir build && cd build

# 2. 生成 Makefile (CMake 会自动读取上面 source 的环境变量)
cmake ..

# 3. 编译源码
make -j$(nproc)

# 4. 将编译好的插件强行注入到 Gazebo 默认的系统目录
sudo make install
```

使用方法:
```
<plugin name="gimbal_ros_control" filename="libGimbalSmall2dPlugin.so">
  <joint>tilt_joint</joint>
  
  <rosServiceName>/uav/gimbal/set_angle</rosServiceName>
  
  <rosTopicName>/uav/gimbal/pitch_status</rosTopicName>
</plugin>
```

## Requirements :
Native Ubuntu able to run full 3D graphics.
Gazebo version 8.x or greater
The dev branch will works on gazebo >= 8.x  
For Gazebo 7 use branch gazebo7

## Disclamer : 
This is a playground until I get some time to push the correct patch to gazebo master (I got hard time to work with mercurial..)!  
So you can expect things to not be up-to-date.  
This assume that your are using Ubuntu 16.04 or Ubuntu 18.04

## Usage :
I assume you already have Gazebo installed with ROS (or without).  
If you don't have it yet, install ROS with sudo apt install ros-melodic-desktop-full 
(follow instruction here http://wiki.ros.org/melodic/Installation/Ubuntu).  
Due to a bug in current gazebo release from ROS, please update gazebo with OSRF version from http://gazebosim.org/tutorials?tut=install_ubuntu

libgazeboX-dev must be installed, X be your gazebo version (9 on ROS melodic).

For Gazebo X
````
sudo apt-get install libgazeboX-dev
````

````
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
````

````
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
````

Set Path of Gazebo Models (Adapt the path to where to clone the repo)
````
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
````

Set Path of Gazebo Worlds (Adapt the path to where to clone the repo)
````
echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
````

````
source ~/.bashrc
````

DONE !

Now launch a world file with a copter/rover/plane and ardupilot plugin, and it should work! 
(I will try to add some world file and model later)

## HELP

How to Launch :  
Launch Ardupilot Software In the Loop Simulation for each vehicle.
On new terminal, Launch Gazebo with basic demo world.

#####ROVER (no model provided for now)

On 1st Terminal(Launch Ardupilot SITL)
````
sim_vehicle.py -v APMrover2 -f gazebo-rover --map --console
````

On 2nd Termianal(Launch Gazebo with demo Rover model)
````
gazebo --verbose worlds/ (Please Add if there is one.)
````

##### COPTER

On 1st Terminal(Launch Ardupilot SITL)
````
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
````

On 2nd Terminal(Launch Gazebo with demo 3DR Iris model)
````
gazebo --verbose worlds/iris_arducopter_runway.world
````

##### PLANE

On 1st Terminal(Launch Ardupilot SITL)
````
sim_vehicle.py -v ArduPlane -f gazebo-zephyr --map --console
````

On 2nd Terminal(Launch Gazebo with demo Zephyr flying wing model)
````
gazebo --verbose worlds/zephyr_ardupilot_demo.world
````

In addition, you can use any GCS of Ardupilot locally or remotely (will require connection setup).
If MAVProxy Developer GCS is uncomportable. Omit --map --console arguments out of SITL launch. and Use APMPlanner 2 or QGroundControl instead.
Local connection with APMPlanner2/QGroundControl is automatic, and recommended.

## Troubleshooting

### Missing libArduPilotPlugin.so... etc 

In case you see this message when you launch gazebo with demo worlds, check you have no error after sudo make install.  
If no error use "ls" on the install path given to see if the plugin is really here.  
If this is correct, check with "cat /usr/share/gazebo/setup.sh" the variable GAZEBO_PLUGIN_PATH. It should be the same as the install path. If not use "cp" to copy the lib to right path. 

For Example

````
sudo cp -a /usr/lib/x86_64-linux-gnu/gazebo-7.0/plugins/ /usr/lib/x86_64-linux-gnu/gazebo-7/
````

path mismatch is confirmed as ROS's glitch. It will be fixed.

### Future(not activated yet)
To use Gazebo gps, you must offset the heading of +90° as gazebo gps is NWU and ardupilot is NED 
(I don't use GPS altitude for now)  
example : for SITL default location
````
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-35.363261</latitude_deg>
      <longitude_deg>149.165230</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>87</heading_deg>
    </spherical_coordinates>
````
Rangefinder
