
# Ros bridge for monoDrive simulator

This ros package aims at providing a simple ros bridge for monodrive simulator.

![rviz setup](./assets/rviz_monodrive_default.png "rviz")
![depthcloud](./assets/depth_cloud_and_lidar.png "depthcloud")



# Features

- [x] Cameras (depth, segmentation, rgb) support
- [x] Add camera matrix
- [x] Lidar sensor support
- [x] Transform publications
- [x] Manual control using ackermann msg
- [x] Autopilot mode using rosparam
- [x] Rosbag in the bridge (in order to avoid rosbag recoard -a small time errors)
- [x] Handle ros dependencies
- [x] Marker/bounding box messages for cars/pedestrian
- [ ] Add traffic light support
- [ ] Support dynamic change (restarting simulation using a topic/rosparam)


# Setup

## Create a catkin workspace and install monodrive_ros_bridge package

### Create the catkin workspace:

    mkdir -p ~/ros/catkin_ws_for_monodrive/src
    cd ~/ros/catkin_ws_for_monodrive
    source /opt/ros/kinetic/setup.bash
    catkin_make
    source ~/ros/catkin_ws_for_monodrive/devel/setup.bash

For more information about configuring a ros environment see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Install monodrive python client in your workspace

    cd monodrive/client
    pip2 install -e .  --user --upgrade   

Check the installation is successfull by trying to import monodrive from python:

    python -c 'import monodrive;print("Success")'

You should see the Success message without any errors.

### Add the monodrive_ros_bridge in the catkin workspace

Run the following command after replacing [PATH_TO_MONODRIVE_CLIENT] with the actual path to monodrive directory on your machine:

    ln -s [PATH_TO_MONODRIVE_CLIENT]/monodrive_ros_bridge/ ~/ros/catkin_ws_for_monodrive/src/
    source ~/ros/catkin_ws_for_monodrive/devel/setup.bash
    rosdep update
    rosdep install --from-paths ~/ros/catkin_ws_for_monodrive
    cd ~/ros/catkin_ws_for_monodrive
    catkin_make

### Add the optional dependencies in the catkin workspace

Run the following commands on your machine:

    cd ~
    sudo apt install libpcap-dev
    git clone https://github.com/monodriveio/monodrive_rviz_plugin
    git clone https://github.com/ros-drivers/velodyne
    ln -s ~/monodrive_rviz_plugin/ ~/ros/catkin_ws_for_monodrive/src/
    ln -s ~/velodyne/ ~/ros/catkin_ws_for_monodrive/src/
    catkin_make

### Add the monodrive module to your python path

Run the following command after replacing [PATH_TO_MONODRIVE_CLIENT] with the actual path to monodrive directory on your machine

    export PYTHONPATH=$PYTHONPATH:[PATH_TO_MONODRIVE_CLIENT]

### Test your installation

Run the tests

    rostest monodrive_ros_bridge ros_bridge_client.test

you should see:

    [monodrive_ros_bridge.rosunit-testTopics/test_publish][passed]

    SUMMARY
     * RESULT: SUCCESS



# Start the ros bridge

Start the ros bridge:

    source ~/ros/catkin_ws_for_monodrive/devel/setup.bash
    roslaunch monodrive_ros_bridge client.launch

To start the ros bridge with rviz use:

    roslaunch monodrive_ros_bridge client_with_rviz.launch

You can setup the wanted camera/sensors in config/settings.yaml.

# Autopilot control

To enable autopilot control set the ros param monodrive_autopilot to True

    rosparam set monodrive_autopilot True

# Manual control

To enable manual control set the ros param monodrive_autopilot to False

    rosparam set monodrive_autopilot False


Then you can send command to the car using the /ackermann_cmd topic.

Example of forward movements, speed in in meters/sec.

     rostopic pub /ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10


Example of forward with steering

     rostopic pub /ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 5.41, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10

  Warning: the steering_angle is the driving angle (in radians) not the wheel angle, for now max wheel is set to 500 degrees.


Example for backward :

     rostopic pub /ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0, steering_angle_velocity: 0.0, speed: -10, acceleration: 0.0,
      jerk: 0.0}" -r 10


# ROSBAG recording

The monodrive_ros_bridge could also be used to record all published topics into a rosbag:

    roslaunch monodrive_ros_bridge client_with_rviz.launch rosbag_fname:=/tmp/save_session.bag

This command will create a rosbag /tmp/save_session.bag

You can of course also use rosbag record to do the same, but using the ros_bridge to do the recording you have the guarentee that all the message are saved without small desynchronization that could occurs when using *rosbag record* in an other process.
