# Start from noetic ros core image
FROM ros:noetic-ros-core

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Install essential packages
RUN apt-get update && apt-get install -y \
    vim git openssh-server libusb-dev synaptic python3-pip \
    python3-termcolor python3-catkin-tools python3-osrf-pycommon \
    ros-noetic-ackermann-msgs ros-noetic-serial ros-noetic-ros-ign \
    ros-noetic-costmap-2d ros-noetic-video-stream-opencv xterm \
    unzip gdb curl python3-tk mjpegtools software-properties-common \
    libv4l-dev inetutils-ping net-tools tmux && \
    apt-get clean

RUN pip3 install timm==0.5.4 protobuf==4.25.3 easydict imageio urdf_parser_py

# Update cmake
RUN apt update && \
    apt install -y software-properties-common lsb-release && \
    apt clean all && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
    apt update && \
    apt install kitware-archive-keyring && \
    rm /etc/apt/trusted.gpg.d/kitware.gpg && \
    apt install -y cmake

# Upgrade Eigen
WORKDIR /root/Software
RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && \
    git checkout 3.4.0 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j $(nproc) && make install && \
    ln -s /usr/local/include/eigen3/Eigen /usr/local/include/Eigen

# Install libtorch
WORKDIR /root/Software
RUN wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu121.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.1.0+cu121.zip

# Install URDFDOM
WORKDIR /root/Software
RUN git clone https://github.com/ros/urdfdom.git && \
    cd urdfdom && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j $(nproc) && make install

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh && \
    apt-get install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Clone and build OpenCV
WORKDIR /root/Software
RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && git checkout 4.5.5 && \
    cd ../opencv_contrib && git checkout 4.5.5 && \
    cd ../opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -D BUILD_EXAMPLES=OFF -D WITH_CUDA=OFF -D WITH_V4L=ON \
          -D WITH_QT=ON -D WITH_OPENGL=ON .. && \
    make -j$(nproc) && make install && ldconfig

# Set OpenCV environment variables
ENV OpenCV_DIR=/usr/local/lib/cmake/opencv4
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Clone and build ORB_SLAM2
WORKDIR /root/Software
RUN git clone https://github.com/raulmur/ORB_SLAM2.git && \
    cd ORB_SLAM2 && \
    sed -i 's/cmake .. /cmake .. -DOpenCV_DIR=\/usr\/local\/lib\/cmake\/opencv4/' build.sh && \
    chmod +x build.sh && \
    ./build.sh || { cat build.sh.log; exit 1; }

# Clone orb_slam2_ros
WORKDIR /root/Software
RUN git clone https://github.com/appliedAI-Initiative/orb_slam2_ros.git && \
    cd orb_slam2_ros/orb_slam2 && \
    ln -s /root/Software/ORB_SLAM2/Thirdparty Thirdparty && \
    ln -s /root/Software/ORB_SLAM2/lib lib && \
    ln -s /root/Software/ORB_SLAM2/Vocabulary Vocabulary && \
    cd .. && mkdir build && cd build && cmake .. && make -j$(nproc)

# ROS Workspace Setup
WORKDIR /root/catkin_ws
RUN mkdir -p src && \
    ln -s /root/Software/orb_slam2_ros src/orb_slam2_ros && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source workspace in bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Final working directory
WORKDIR /root
