# System Configuration

## Turtlebot

### 1. Install dependencies

```console
$ python3 -m pip install imagezmq
```

### 2. Move the camera node into the turtlebot3_bringup project
Move or copy the file 'src/eye.py' from this project to 'turtlebot3_bringup/src/eye.py' on the turtlebot.


### 3. Modify the launch file to include the camera node
Insert the following line into 'turtlebot3_bringup/launch/turtlebot3_robot.launch'

```xml
    <node pkg="turtlebot3_bringup" type="eye.py" name="eye"/>
```

### 4. Run the Camera node on the Turtlebot 
Note: make sure $ROS_MASTER_URI is correctly set on the turtlebot. eye.py uses ROS_MASTER_URI to connect to receiver
Only 1 reciever can receive camera images when compared to ROS, where every computer on the ROS network can get the camera images.

Run the camera as python script

```console
$ roscd turtlebot3_bringup/src
$ python3 eye.py
```

OR 

Use 'bringup' to launch all nodes on the Turtlebot

```console
$ bringup
```

## Host

### 1. Install dependencies

```console
$ python3 -m pip install imagezmq
```

### 2. Run the reciever as a python script

```console
$ python3 viewVideo.py
```
