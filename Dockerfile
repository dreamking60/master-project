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

# Build ORB-SLAM3 with Pangolin visualization components
RUN chmod +x build.sh build_ros.sh && \
    ./build.sh && ./build_ros.sh

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