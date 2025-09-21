FROM aflplusplus/aflplusplus

WORKDIR  /home

# Set timezone to avoid interactive prompt
ENV TZ=America/Chicago
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# install dependencies for ORB-SLAM3 including Pangolin
RUN apt-get update && apt-get install -y \
    curl \
    cmake \
    build-essential \
    git \
    pkg-config \
    clang lld \
    libeigen3-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    libboost-all-dev \
    libssl-dev \
    libfmt-dev \
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libepoxy-dev \
    && rm -rf /var/lib/apt/lists/*

ENV CC=clang
ENV CXX=clang++

# Build and install Pangolin first
WORKDIR /home
COPY Pangolin/ Pangolin/
WORKDIR /home/Pangolin
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Wno-error=missing-braces -Wno-error=type-limits" && \
    make -j4 && \
    make install && \
    ldconfig
    
# Copy and build ORB-SLAM3 elastic library with Pangolin support
WORKDIR /home
COPY orb-slam-elastic/ orb-slam-elastic/
WORKDIR /home/orb-slam-elastic

# Fix OpenCV version check inconsistency in CMakeLists.txt
RUN sed -i 's/find_package(OpenCV 3.0)/find_package(OpenCV 4.0)/' CMakeLists.txt
RUN sed -i 's/-march=native/-mtune=generic/g' CMakeLists.txt
RUN sed -i 's/-march=native/-mtune=generic/g' Thirdparty/g2o/CMakeLists.txt
RUN sed -i 's/-march=native/-mtune=generic/g' Thirdparty/DBoW2/CMakeLists.txt
RUN sed -i 's/-march=native/-mtune=generic/g' Examples/ROS/OO_SLAM3/CMakeLists.txt
RUN sed -i 's/-march=native/-mtune=generic/g' Examples_old/ROS/ORB_SLAM3/CMakeLists.txt
RUN sed -i 's|<stdint-gcc.h>|<cstdint>|' src/ORBmatcher.cc
RUN sed -i 's|<stdint-gcc.h>|<cstdint>|' Thirdparty/DBoW2/DBoW2/FORB.h
RUN sed -i 's|<stdint-gcc.h>|<cstdint>|' Thirdparty/DBoW2/DBoW2/FORB.cpp
RUN sed -i '/#include <utility>/a #include <iostream>' include/ImuTypes.h

# Replace C++11/C++0x support check with C++14 standard
RUN sed -i '/# Check C++11 or C++0x support/,/endif()/c\# Set C++14 standard\
set(CMAKE_CXX_STANDARD 14)\
set(CMAKE_CXX_STANDARD_REQUIRED ON)\
set(CMAKE_CXX_EXTENSIONS OFF)' CMakeLists.txt

# Modify CMakeLists.txt to add warning suppression flags
#RUN sed -i 's/set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")/set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fsigned-char -Wno-error -Wno-aggressive-loop-optimizations -Wno-class-memaccess -Wno-maybe-uninitialized")/' CMakeLists.txt

# Build ThirdParty libraries first
RUN echo "Building ThirdParty/DBoW2..." && \
    cd Thirdparty/DBoW2 && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j4

RUN echo "Building ThirdParty/g2o..." && \
    cd Thirdparty/g2o && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j4

RUN echo "Building ThirdParty/Sophus..." && \
    cd Thirdparty/Sophus && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j4

# Build ORB-SLAM3 core library first
RUN chmod +x build.sh build_ros.sh && \
    ./build.sh

# Install dependencies for ROS Noetic
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add deadsnakes PPA for Python 3.8
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y \
    python3.8 \
    python3.8-dev \
    python3.8-venv \
    python3.8-distutils \
    libpython3.8-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for python3.8 as python3
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Install pip for Python 3.8
RUN curl -sS https://bootstrap.pypa.io/pip/3.8/get-pip.py | python3.8

# Install ROS Python dependencies
RUN python3.8 -m pip install -U rosdep rosinstall rosinstall-generator wstool

# Add Ubuntu 20.04 (Focal) repositories for missing packages (ARM64 compatible)
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports focal main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://ports.ubuntu.com/ubuntu-ports focal-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://ports.ubuntu.com/ubuntu-ports focal-security main restricted universe multiverse" >> /etc/apt/sources.list

# Add ROS repository for Ubuntu 20.04 (Focal) - ARM64 compatible
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros.list > /dev/null

# Update package lists
RUN apt-get update

# Create package preferences to prioritize Ubuntu 20.04 packages
RUN echo "Package: *" > /etc/apt/preferences.d/99focal && \
    echo "Pin: release a=focal" >> /etc/apt/preferences.d/99focal && \
    echo "Pin-Priority: 500" >> /etc/apt/preferences.d/99focal && \
    echo "" >> /etc/apt/preferences.d/99focal && \
    echo "Package: libboost-*" >> /etc/apt/preferences.d/99focal && \
    echo "Pin: release a=focal" >> /etc/apt/preferences.d/99focal && \
    echo "Pin-Priority: 1000" >> /etc/apt/preferences.d/99focal && \
    echo "" >> /etc/apt/preferences.d/99focal && \
    echo "Package: libopencv-*" >> /etc/apt/preferences.d/99focal && \
    echo "Pin: release a=focal" >> /etc/apt/preferences.d/99focal && \
    echo "Pin-Priority: 1000" >> /etc/apt/preferences.d/99focal

# Install complete OpenCV 4.2 and Boost 1.71 packages from Ubuntu 20.04
RUN apt-get install -y \
    libopencv-dev=4.2.0+dfsg-4ubuntu0.1 \
    libopencv-contrib-dev=4.2.0+dfsg-4ubuntu0.1 \
    libboost-all-dev=1.71.0.0ubuntu2 \
    libconsole-bridge0.4 \
    libyaml-cpp0.6 \
    libprotobuf17 \
    libtinyxml2-6a \
    liborocos-kdl1.4 \
    liburdfdom-model \
    liburdfdom-world \
    liblog4cxx10v5 \
    libpocofoundation62 \
    libpcl-common1.10 \
    libpcl-features1.10 \
    libpcl-filters1.10 \
    libpcl-io1.10 \
    libpcl-search1.10 \
    libpcl-segmentation1.10 \
    libpcl-surface1.10 \
    hddtemp \
    || true

# Install ROS Noetic core packages (ARM64 compatible)
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-core \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-geometry \
    ros-noetic-camera-calibration-parsers \
    ros-noetic-rosbag \
    ros-noetic-rosbag-storage \
    ros-noetic-rosconsole \
    ros-noetic-rosconsole-bridge \
    ros-noetic-roscpp-tutorials \
    ros-noetic-rospack \
    ros-noetic-rostopic \
    ros-noetic-rosservice \
    ros-noetic-rosnode \
    ros-noetic-rosmsg \
    ros-noetic-roslaunch \
    ros-noetic-rosmaster \
    ros-noetic-rosout \
    ros-noetic-rosgraph \
    ros-noetic-rosgraph-msgs \
    ros-noetic-roslib \
    ros-noetic-rostest \
    ros-noetic-rosbuild \
    ros-noetic-rosmake \
    ros-noetic-rosbash \
    ros-noetic-roscreate \
    ros-noetic-roslang \
    ros-noetic-roslz4 \
    ros-noetic-rosparam \
    ros-noetic-roswtf \
    || true

# Initialize rosdep (with error handling for ARM64)
RUN rosdep init || true && rosdep update --rosdistro noetic || true

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV ROS_PACKAGE_PATH=/home/orb-slam-elastic/Examples_old/ROS:/opt/ros/noetic/share
ENV ROS_WORKSPACE=/home/orb-slam-elastic/Examples_old/ROS

# Build ROS nodes after ROS is installed
RUN . /opt/ros/noetic/setup.sh && \
    export ROS_PACKAGE_PATH=/home/orb-slam-elastic/Examples_old/ROS:/opt/ros/noetic/share && \
    cd /home/orb-slam-elastic/Examples_old/ROS/ORB_SLAM3 && \
    rospack profile && \
    # Fix OpenCV version requirement in CMakeLists.txt
    sed -i 's/find_package(OpenCV 3.0 REQUIRED)/find_package(OpenCV 4.0 REQUIRED)/' CMakeLists.txt && \
    sed -i 's/message(FATAL_ERROR "OpenCV > 4.4 not found.")/message(STATUS "OpenCV version: ${OpenCV_VERSION}")/' CMakeLists.txt && \
    # Fix C++ standard to C++14 for Pangolin compatibility
    sed -i '/cmake_minimum_required/a set(CMAKE_CXX_STANDARD 14)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)' CMakeLists.txt && \
    ldconfig && \
    mkdir -p build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 && \
    make -j4

# install rust nightly with workaround for cross-device link issue
WORKDIR /home
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path && \
    rustup toolchain install nightly && \
    rustup default nightly

# install cargo-fuzz
RUN cargo install cargo-fuzz

# Create Rust example
WORKDIR /home/rust-example
RUN cargo init
RUN cargo fuzz init
COPY fuzz_safe_copy.rs fuzz/fuzz_targets/fuzz_target_1.rs

# Create C example (moved to last)
WORKDIR /home
RUN mkdir -p c-example/in
COPY fuzz_simple.c c-example/
RUN afl-clang-fast -o c-example/fuzz_simple c-example/fuzz_simple.c
RUN echo "test-stevenchen" > c-example/in/test

# Back to home
WORKDIR /home
CMD ["sh", "-c", "echo 'AFL++ + Rust fuzzing environment with ORB-SLAM3 is ready!'; \
     echo 'Run C fuzzer: afl-fuzz -i c-example/in -o c-example/out -- c-example/fuzz_simple @@'; \
     echo 'Run Rust fuzzer: cd rust-example && cargo fuzz run fuzz_target_1'; \
     echo 'ORB-SLAM3 elastic library with Pangolin is built and available in: /home/orb-slam-elastic/'; \
     exec sh"]