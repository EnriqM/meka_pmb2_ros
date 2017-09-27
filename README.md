# meka_pmb2_ros
Final project joining functions and make Meka and PMB2 robots to cooperate, full in ROS.

A video showing how it works: [Video of demo_smach working](https://youtu.be/lONYOklIyIk)   

### Installation

Just copy all the packages in the "src" folder of a catkin workspace. Then compile

### Dependencies

**Install mandatory dependencies**

```
sudo apt-get install \
    ros-<distro>-laser-filters \
    ros-<distro>-openni-launch \
    ros-<distro>-openni-* \
    ros-<distro>-smach-* \ 
    ros-<distro>-gmapping \
    ros-<distro>-navigation \
    ros-<distro>-moveit \
    ros-<distro>-laser-filters
    libopenni0 \
    libopenni-dev \
    python-termcolor
```

**Install packages**

Add the git repository to your catkin workspace:

```
cd <your-catkin-ws>/src
git clone https://github.com/EnriqM/meka_pmb2_ros.git
```

*Hint*: If the Python scripts don't work, probably you'll have to give them permissions:    
``chmod u+x *`` 

**Compile**
 
To compile, you can now use catkin\_make as usual:

```
cd ..
catkin_make
```
