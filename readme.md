Installing docker-compose in TX2:
put the follwing lines into install the prerequisite packages, as there is no docker compose or arm64 architecture and needs to be installed separately through pip
```
export DOCKER_COMPOSE_VERSION=1.27.4
sudo apt-get install libhdf5-dev
sudo apt-get install libssl-dev
sudo apt install python3
sudo apt install python3-pip
sudo pip3 install docker-compose=="${DOCKER_COMPOSE_VERSION}"
pip install docker-compose
```

Switch directory to 
```
cd /home/ubuntu/docker_rover_driver/Rover_driver_test
```
Build Command:
check that the hostname is correct in the docker file.
```
    docker build -t rover_driver .
```
Run Command:
```
docker run --rm -it --net=host --device=/dev/ttyACM0 rover_driver
```

Testing:
First source
```source devel/setup.bash```
then roslaunch 
``` roslaunch roboclaw_node roboclaw.launch```
Launch roscore on laptop/host

Note: Communication pipeline is ros2/joystick->bridge->ros1/container->tx2/roboclaw, have ros1 to roboclaw working, need to get commands from outside ros1 container to inside it./

Ros1-Ros2 Bridge:
Make sure ros1 container uses a launch file so that roscore starts, as it needs a master. 

Note: DON'T source both ros1 and ros2 in same terminal

Note: 
Manually publish message to ros2:
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
To ssh into rover7 while they are both on swarm wifi: 
```
ssh ubuntu@rover7.local
```
and then clone the repo if it already is not. Change directory into the main folder and run '''docker-compose build''' and '''docker-compose up'''

Testing
Then open up the ground station docker-compose file. Go into the px4_ros_com folder and run '''colcon build''' 

TODO:
Vicon receiver site: https://github.com/OPT4SMART/ros2-vicon-receiver
need to add something to source and run the vicon receiver

$ source vicon_receiver/install/setup.bash
$ ros2 launch vicon_receiver client.launch.py

this will make sure the vicon data is accessible

in another docker container cd into the rover waypoint package and source and then build it seperately

to run the move_waypoint use

'''
ros2 run rover_waypoint move_waypoint
'''

cmd_vel publisher works, publishing waypoint seems to work for finding location

for now in a separate container, publish waypoint to test single waypoint

'''
ros2 topic pub -1 /des_wp geometry_msgs/msg/Pose '{position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {x: 1.0, y: 0.0, z: 0.0 ,w: 1.0}}'
'''

TODO: Check that the waypoints are being correctly converted and that gains are sufficiently low, is moving way to fast with way too much distance