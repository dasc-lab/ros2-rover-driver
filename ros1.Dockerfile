FROM ros:melodic
#melodic is the latest version that runs on python 2, cannot use noetic.

RUN apt-get -y update
RUN apt-get -y install git
RUN sudo apt-get -y install ros-melodic-tf
RUN sudo apt -y install python-pip
RUN sudo apt -y install ros-melodic-diagnostics
RUN pip install pyserial
# RUN sudo apt install -y iproute2

RUN sudo apt-get install -y tmux iputils-ping

WORKDIR /root/

# SHELL [ "bash" ]

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN mkdir -p ~/catkin_ws/src 
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make'
WORKDIR /root/catkin_ws/src
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN git clone https://github.com/dasc-lab/rover-driver.git
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ..; catkin_make'

# Shell file to source and launch roboclaw
COPY ./ros_entrypoint.sh .
RUN chmod +x ros_entrypoint.sh

####################
###### NOTE! MUST CHANGE IP ADDRESS TO MATCH THAT OF THE ROSMASTER COMPUTER
####################

#ENV ROS_MASTER_URI http://dasc1.local:11311
# ENV ROS_MASTER_URI http://192.168.1.122:11311 
# ENV ROS_HOSTNAME rover7.local

CMD ["./ros_entrypoint.sh"]
