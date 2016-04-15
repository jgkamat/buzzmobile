# buzzmobile
An autonomous parade float/vehicle


Environment
-----------

To get started, be running Ubuntu 14.04 (required for ROS Indigo).
Install ROS Indigo by following the tutorial [here](http://wiki.ros.org/indigo/Installation/Ubuntu).

It's a good idea to set up some helpful aliases in your `.bashrc`:

```bash
alias rosinit='source /opt/ros/indigo/setup.bash'
alias rosdevel='source ~/catkin_ws/devel/setup.bash' # needed for autocompletion

rosinit     # put these in your .bashrc too so they run everytime
rosdevel    # you open a new terminal window/tab.
```

Buzzmobile current requires [OpenCV 3.1.0](https://github.com/Itseez/opencv/archive/3.1.0.zip),
so you must install that by extracting that `.zip`, then running:

```bash
cd opencv_3.1.0
mkdir build
cd build
cmake ..
make -j4
sudo make install     # this part may take a while.
```


Running
-------

To start running, you must first run `catkin_make` from the `~/catkin_ws` dir.
If there aren't any issues with the build, you can run `roscore` to start the
main ROS process.

Create new tabs for every node and run them as such:

```rosrun buzzmobile image_const```

```rosrun buzzmobile edge_detector```
