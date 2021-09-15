## Full integration of self driving car system in ROS
### Overview
The aim of this project is to integrate and simulate different functionality of the self driving car algrothim in Robotics Operating System (ROS). The self driving car system takes the following configuration:- 

![Alt text](imgs/final-project-ros-graph-v2.png?raw=true "Overview")

As it can be seen above the system is composed of saveral subsystems. Perception, planning and control are the main components that represent the self driving car algorthim. A unity simulator is used to simulate the car behavior as well as making use of a virtual environment, it is needed to interface with the above system as it shown above. A link of the simulator can be found [here]https://github.com/udacity/CarND-Capstone/releases


### Perception 

  [tl_detector.py](/ros/src/tl_detector/tl_detector.py)  
  
  <img src="imgs/tl-detector-ros-graph.png" width="700" height="180" />
  
This subsystem is composed of obstacle detection and traffic light detection. The traffic light detection can be obtained from the simulator, however, another method would be to implement traffic light classification algorithm. The input to this node is `/image_color`, `/current_pose` of the car, and the `/base_waypoints`. The base waypoints can be thought as path planning points on the road for the car to follow, while the current pose represent the position of the car at that instance. These are all topics that this node subscribes to, in order to process this information and publish to the  `/traffic_waypoint` topic. 



### Planning  

#### Waypoint Loader
A script which loads static waypoint data and publishes to `/base_waypoints`.

#### Waypoint Updater
[wayppoint_updater.py](/ros/src/waypoint_updater/waypoint_updater.py)

<img src="imgs/waypoint-updater-ros-graph.png" width="700" height="180" />

With current pose given from the simulator, the planner will compare that to the loaded waypoints and generate base waypoints to the waypoint updater. The waypoint updater then will send final wapoints to the control to be followed. Moreover, traffic light and obstacle information are taken from the preception node to ensure that the car will not run a red light or crash into an obstacle. This node subscribes to  `/current_pose` ,`/base_waypoints` and `/traffic_waypoint` topics. Then it publishes the list of waypoints aheaad with the information of a velocity to follow (target velocity) to the `/final_waypoints` topic.


### Control 

The control subsystem under (/ros/src/twist_controller) can be mainly decribed by the following python files:- 
1. Waypoint follower script: subscribes to the `/final_waypoints` (from the waypoint updater) and publishes target linear and angular velocities in the `/twist_cmd` topic.
2. The Drive by wire (DBW) script: defines the DBW object parameters , subscribes to the `/twist_cmd` topic and publishes the three control commands (throttle, steering and brake). 
3. The Twist controller script: defines the controller object and  outputs the throttle, brake and steering commands.
4. low pass filter script: Used to smooth the data coming into the controller.  
5. PID controller script: Used to maintain a robust and stable control input into the car's used for throttle controller.
6. the yaw controller script: gives out the steering command to the twist controller script.  


#### Drive By Wire 

 [dbw_node.py](/ros/src/twist_controller/dbw_node.py) 
 
 <img src="imgs/dbw-node-ros-graph.png" width="700" height="180" />


One of the main tasks of this node is transforming the commanded waypoints from waypoint updated to brake, thrttole and steering commands. This node subscribes to `/current_velocity` topic and `/twist_cmd` topic. The difference between the current state of the car and the commanded state will then be transformed into throttle, brake and steering commands. Moreover, this node also subcribes to `/vehicle/dbw_enabled` topic to differentiate between manual mode or self driving mode.  

The output of this node is the three input controls to the car under the topics: `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.


### Running the full ROS system 

I have created a bash script (Run.bash)  that deletes the build folder (if exists) then catkin_make the ros scripts, then sources the `/devel/setup.sh`, then finally runs the `roslaunch /launch/styx.launch`

`rm -r build/ ; catkin_make; source devel/setup.sh ; roslaunch launch/styx.launch`

This bash file needs to be tranformed into excutable before running it `chmod +x ./Run.bash` . 

## Installations

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
*    If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
