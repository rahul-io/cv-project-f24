# Start from noetic ros 20 image
FROM ros:noetic-ros-core


### ROS

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# apt update
RUN apt-get update

# apt install
RUN apt-get install -y \
    vim \
    git \
    openssh-server \
    libusb-dev \
    synaptic \
    python3-pip \
    python3-termcolor \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-ackermann-msgs \
    ros-noetic-serial \
    ros-noetic-ros-ign \
    ros-noetic-costmap-2d \
    ros-noetic-video-stream-opencv \
    xterm \
    unzip \
    gdb \
    curl \
    python3-tk \
    mjpegtools \
    software-properties-common \
    libv4l-dev \
    inetutils-ping \
    net-tools \
    tmux

RUN pip3 install timm==0.5.4 protobuf==4.25.3 easydict imageio urdf_parser_py

### SOFTWARE DEPENDENCIES

RUN export cores=$(nproc)

# update cmake
WORKDIR /root/Software
RUN apt update
RUN apt install -y software-properties-common lsb-release
RUN apt clean all
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
RUN apt update
RUN apt install kitware-archive-keyring
RUN rm /etc/apt/trusted.gpg.d/kitware.gpg
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA
RUN apt update
RUN apt install -y cmake

# upgrade eigen
WORKDIR /root/Software
RUN git clone https://gitlab.com/libeigen/eigen.git
WORKDIR /root/Software/eigen
RUN git checkout 3.4.0
RUN mkdir build
WORKDIR /root/Software/eigen/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j $cores && sudo make install
WORKDIR /usr/local/include
RUN ln -s /usr/local/include/eigen3/Eigen Eigen

# install torchlib
WORKDIR /root/Software
RUN wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu121.zip
RUN unzip libtorch-cxx11-abi-shared-with-deps-2.1.0+cu121.zip
RUN echo "export LIBTORCH_PATH=/root/Software/libtorch" >> ~/.bashrc

# install urdfdom
WORKDIR /root/Software
RUN git clone https://github.com/ros/urdfdom.git
WORKDIR /root/Software/urdfdom
RUN mkdir build
WORKDIR /root/Software/urdfdom/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev
RUN make -j $cores && sudo make install

# install gazebo
WORKDIR /root/Software
RUN curl -sSL http://get.gazebosim.org | sh
RUN apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# PAC-NMPC ML/RL Setup
# WORKDIR /root 
# ADD Software/python Projects/pacnmpc/pacnmpc_python
# RUN rm -rf /usr/lib/python3.8/site-packages/ ;
# RUN ln -s /usr/local/lib/python3.8/dist-packages /usr/lib/python3.8/site-packages
# RUN pip3 install setuptools==65.6.0
# RUN pip3 install -r /root/Projects/pacnmpc/pacnmpc_python/requirements.txt
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
# RUN pip3 install -e /root/Projects/pacnmpc/pacnmpc_python/pacnmpc_rl
# RUN pip3 install -e /root/Projects/pacnmpc/pacnmpc_python/dependencies/fsrl
# WORKDIR /root
# RUN rm -rf Projects

# make projects directory
WORKDIR /root
RUN mkdir Projects

# set up aliases
# RUN echo 'alias pacnmpc-ros-cd="cd /root/Projects/pacnmpc_ws"\nalias pacnmpc-ros-build="pacnmpc-ros-cd && catkin build && source devel/setup.bash"' >> ~/.bashrc_aliases
# RUN echo 'source ~/.bashrc_aliases' >> ~/.bashrc
# ENV QT_DEBUG_PLUGINS=1

WORKDIR /root
