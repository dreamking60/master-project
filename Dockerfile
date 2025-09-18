FROM aflplusplus/aflplusplus

WORKDIR  /home

# Set timezone to avoid interactive prompt
ENV TZ=America/Chicago
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# install dependencies for ORB-SLAM3 and Pangolin
RUN apt-get update && apt-get install -y \
    curl \
    cmake \
    build-essential \
    git \
    pkg-config \
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
    libepoxy-dev \
    libopenexr-dev \
    libilmbase-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff5-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libwayland-dev \
    && rm -rf /var/lib/apt/lists/*

# Build Pangolin from our submodule with char fix
COPY Pangolin/ /tmp/Pangolin/
RUN cd /tmp/Pangolin && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_CXX_FLAGS="-fsigned-char -Wno-error" && \
    make -j4 2>&1 | tee /home/pangolin_build.log && \
    make install && \
    echo "Pangolin built with signed char fix. Build log saved to /home/pangolin_build.log" && \
    cd / && rm -rf /tmp/Pangolin

# Pangolin is now built and installed from submodule

# install rust nightly with workaround for cross-device link issue
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path && \
    rustup toolchain install nightly && \
    rustup default nightly

# install cargo-fuzz
RUN cargo install cargo-fuzz

# Creat C example
RUN mkdir -p c-example/in
COPY fuzz_simple.c c-example/
RUN afl-clang-fast -o c-example/fuzz_simple c-example/fuzz_simple.c
RUN echo "test-stevenchen" > c-example/in/test


# Create Rust example
WORKDIR /home/rust-example
RUN cargo init
RUN cargo fuzz init
COPY fuzz_safe_copy.rs fuzz/fuzz_targets/fuzz_target_1.rs

# Copy and build ORB-SLAM3 elastic library
WORKDIR /home
COPY orb-slam-elastic/ orb-slam-elastic/
WORKDIR /home/orb-slam-elastic

# Build ORB-SLAM3
RUN chmod +x build.sh build_ros.sh && \
    ./build.sh && \
    ./build_ros.sh

# Back to home
WORKDIR /home
CMD ["sh", "-c", "echo 'AFL++ + Rust fuzzing environment with ORB-SLAM3 is ready!'; \
     echo 'Run C fuzzer: afl-fuzz -i c-example/in -o c-example/out -- c-example/fuzz_simple @@'; \
     echo 'Run Rust fuzzer: cd rust-example && cargo fuzz run fuzz_target_1'; \
     echo 'ORB-SLAM3 elastic library is built and available in: /home/orb-slam-elastic/'; \
     exec sh"]