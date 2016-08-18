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

We recommend using OpenCV 3. You can install it with:

```bash
sudo apt-get install ros-indigo-opencv3
```

To set up this repo, do:

```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Now you can clone this repo into `~/catkin_ws/src` and run `rosdep install buzzmobile` to install some dependencies, like [usb_cam].


Running
-------

To start running, you must first run `catkin_make` from the `~/catkin_ws` dir.
If there aren't any issues with the build, you can run `roscore` to start the
main ROS process.

Create new tabs for every node and run them as such:

```rosrun buzzmobile image_const```

```rosrun buzzmobile edge_detector```

```rosparam set usb_cam/pixel_format yuyv
rosrun usb_cam usb_cam_node```

If you want to visualize your nodes, you can run the ROS visualizer.

```bash
rosrun rviz rviz
```

In it, you can, for instance, create an 'image' instance, and set it to the
value of the `/image_const` being broadcast. This will display the image.
