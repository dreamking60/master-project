FROM aflplusplus/aflplusplus

WORKDIR  /home

# install dependencies
RUN apt-get update && apt-get install -y curl

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

# Back to home
WORKDIR /home
CMD ["sh", "-c", "echo 'AFL++ + Rust fuzzing environment is ready!'; \
     echo 'Run C fuzzer: afl-fuzz -i c-example/in -o c-example/out -- c-example/fuzz_simple @@'; \
     echo 'Run Rust fuzzer: cd rust-example && cargo fuzz run fuzz_target_1'; \
     exec sh"]