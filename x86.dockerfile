FROM curobo_ros:ada_lovelace-dev

# Add camera azure kinect
RUN apt update && apt install software-properties-common \
                    ros-humble-librealsense2* \
                    ros-humble-realsense2-* -y &&\
                    apt-add-repository -y -n 'deb http://archive.ubuntu.com/ubuntu focal main' && \
                    apt-add-repository -y 'deb http://archive.ubuntu.com/ubuntu focal universe'
RUN apt-get install -y libsoundio1
RUN apt-add-repository -r -y -n 'deb http://archive.ubuntu.com/ubuntu focal universe' && \
    apt-add-repository -r -y 'deb http://archive.ubuntu.com/ubuntu focal main'


RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb > /tmp/libk4a1.3_1.3.0_amd64.deb
RUN echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections
RUN sudo dpkg -i /tmp/libk4a1.3_1.3.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb > /tmp/libk4a1.3-dev_1.3.0_amd64.deb
RUN sudo dpkg -i /tmp/libk4a1.3-dev_1.3.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb > /tmp/libk4abt1.0_1.0.0_amd64.deb
RUN echo 'libk4abt1.0	libk4abt1.0/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections
RUN sudo dpkg -i /tmp/libk4abt1.0_1.0.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb > /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
RUN sudo dpkg -i /tmp/libk4abt1.0-dev_1.0.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb > /tmp/k4a-tools_1.3.0_amd64.deb
RUN sudo dpkg -i /tmp/k4a-tools_1.3.0_amd64.deb

COPY 99-k4a.rules /etc/udev/rules.d/99-k4a.rules

# Add realsense lib
# RUN apt install 

# Add azure, doosan, tool_box and pcd_fuse ros2 package
WORKDIR /home/ros2_ws/src

# Add point cloud fusion
RUN git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git && \
    git clone -b humble https://github.com/ros-controls/gz_ros2_control && \
    git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git 

RUN sed -i '771d' /home/ros2_ws/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp

### install gazebo sim for doosan package
# RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# RUN apt-get update
# RUN apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs\
#         ros-humble-ros-gz-sim ros-humble-ros-gz libpoco-dev libyaml-cpp-dev\
#         ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro\
#         ros-humble-joint-state-publisher-gui ros-humble-ros2-control\
#         ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs\
#         dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
#         libignition-gazebo6-dev libignition-msgs9-dev libignition-common5-dev \
#         ros-humble-image-proc
RUN sudo apt-get update && apt-get install -y libpoco-dev libyaml-cpp-dev wget \
                        ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
                        ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
                        ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
                        dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
                        ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control

RUN  sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ros-gz




### Install Field2Cover
RUN  apt-get update && \
    apt-get install --no-install-recommends -y ca-certificates \
    doxygen git libeigen3-dev libgdal-dev    \
    python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev \
    gnuplot libtinyxml2-dev nlohmann-json3-dev && python3 -m pip install gcovr

RUN cd /home/ && git clone https://github.com/Fields2Cover/Fields2Cover.git
RUN cd /home/Fields2Cover && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install 

WORKDIR /home/ros2_ws

# Fix protobuf version conflict - ensure system protobuf is used after Fields2Cover installation
RUN rm -rf /usr/local/bin/protoc* \
           /usr/local/lib/libprotobuf* \
           /usr/local/lib/cmake/protobuf* \
           /usr/local/lib/cmake/utf8_range* \
           /usr/local/include/google/protobuf* && \
    apt-get install -y --reinstall libprotobuf-dev protobuf-compiler && \
    ldconfig

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"
