# Start from noetic ros core image
FROM ros:noetic-ros-core

# Setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Add any necessary dependencies here
# NOTE: if you need to add more dependencies, create a separate apt-get line while you are testing, so that you don't have to re-install all of these every time.
# Once your library successfully builds, then consolidate the two apt-get lines together.
RUN apt-get update && apt-get install -y \
    # Basic Tools
    vim \
    git \
    wget \
    curl \
    tmux \
    xterm \
    gdb \
    unzip \
    net-tools \
    inetutils-ping \
    software-properties-common \
    # Development Tools
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    python3-tk \
    python3-termcolor \
    python3-catkin-tools \
    python3-osrf-pycommon \
    # Libraries
    libusb-dev \
    libv4l-dev \
    libeigen3-dev \
    libboost-all-dev \
    libepoxy-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    libpcl-dev \
    libsuitesparse-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libyaml-cpp-dev \
    # ROS Noetic Dependencies
    ros-noetic-ackermann-msgs \
    ros-noetic-serial \
    ros-noetic-ros-ign \
    ros-noetic-costmap-2d \
    ros-noetic-tf2 \
    ros-noetic-pcl-ros \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-vision-opencv \
    ros-noetic-rosbridge-suite \
    ros-noetic-catkin \
    # Miscellaneous Tools
    mjpegtools \
    openssh-server \
    synaptic \
    && rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Add any necessary Python dependencies here
RUN pip3 install --upgrade pip
RUN pip3 install numpy scipy matplotlib opencv-python pybind11 picamera2

# # Install libtorch
# WORKDIR /root/Software
# RUN wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu121.zip && \
#     unzip libtorch-cxx11-abi-shared-with-deps-2.1.0+cu121.zip

# Install Pangolin
WORKDIR /root/Software
RUN cd ~ && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && \
    cmake .. && make -j4 && make install
    
# THIS is the end of dependencies. The code below is for each of the SLAM libraries.

######################################################################################

## ORBSLAM3
# Install Orb_SLAM3
WORKDIR /root/catkin_ws
RUN mkdir -p src && cd src && \
    git clone https://github.com/thien94/orb_slam3_ros.git && \ 
    cd ../ && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -j4"


## DSO
# Clone DSO SLAM repository
RUN cd /root/catkin_ws/src && \
    git clone --recursive https://github.com/JakobEngel/dso.git && \
    cd dso && mkdir build && cd build && cmake .. && make -j4

# Clone the dso_ros wrapper repository
RUN cd /root/catkin_ws/src && \
    git clone --recursive https://github.com/JakobEngel/dso_ros.git


######################################################################################

# Source workspace in bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
    
# Final working directory
WORKDIR /root