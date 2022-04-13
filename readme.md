Installing docker-compose in TX2:
put the follwing lines into install the prerequisite packages, as there is no docker compose or arm64 architecture and needs to be installed separately through pip
```
export DOCKER_COMPOSE_VERSION=1.27.4
sudo apt-get install libhdf5-dev
sudo apt-get install libssl-dev
apt install python3
apt install python3-pip
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
```source devel\setup.bash```
then roslaunch 
``` roslaunch roboclaw_node roboclaw.launch```
Launch roscore on laptop/host

Note: Communication pipeline is ros2/joystick->bridge->ros1/container->tx2/roboclaw, have ros1 to roboclaw working, need to get commands from outside ros1 container to inside it./

Ros1-Ros2 Bridge:
Make sure ros1 container uses a launch file so that roscore starts, as it needs a master. 