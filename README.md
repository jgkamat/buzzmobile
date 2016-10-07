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

To use the google maps api, you need an api key. Put it under `buzzmobile/sense/gps/googlemapskey.py` like such:

```python
googlemapskey='your_secret_api_key'
```


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

Note that rospy nodes don't require `catkin_make`, but do require that the file be made executable, and the `.py` extension:

```bash
chmod +x path/to/rospy_node.py  # also make sure the file has the correct python shebang
rosrun rospy_node.py
```

If you want to visualize your nodes, you can run the ROS visualizer or image_view.

```bash
rosrun rviz rviz
rosrun image_view image_view image:=some_imgmsg
```

In it, you can, for instance, create an 'image' instance, and set it to the
value of the `/image_const` being broadcast. This will display the image.


Recording
---------

If you want to record the messages being outputted by certain nodes, you can use rosbag:

```
mkdir ~/bagfiles
cd ~/bagfiles
rosbag record -O filename /message/name
```

To see info about the recorded data, do `rosbag info filename.bag`

To play the data (and publish those messages), do `rosbag play test.bag`

To see your video (if you recorded camera data), do `rqt_image_view` (rqt comes with ROS if your download the desktop version), play the data if it isn't already playing, hit the refresh button at the top, and then search for the topic your .bag file is publishing to. Your camera data will play in that window.
