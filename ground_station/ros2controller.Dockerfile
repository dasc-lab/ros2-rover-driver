FROM chenrc98/vicon_px4_ros2_bridge:version1.0

#http://manpages.ubuntu.com/manpages/bionic/man1/jstest-gtk.1.html
RUN sudo apt-get -y update
RUN sudo apt-get -y install python3-numpy python3-scipy
RUN sudo apt -y install jstest-gtk

CMD ["bash"]