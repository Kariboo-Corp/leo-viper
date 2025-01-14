# Installation

```bash
docker build -t ros2 .
docker run -it --rm -v $(pwd)/src:/home/ws/src/arm_integration ros2 bash
```	

> Note1 : Make sure you are in the repository root directory before running the above commands.

> Note2 :  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  can be added to the docker run command to enable GUI support (For linux), on Windows, you can use VcXsrv.
>

# Usage

Make sure to source ROS2
```bash
source /opt/ros/humble/setup.bash
```

Then go to the ws directory and build the package
```bash
cd /home/ws
colcon build --symlink-install
```
Then  you can run 
```bash
ros2 launch arm_integration leo_gz.launch.py 
```
and see if everything is working fine.
