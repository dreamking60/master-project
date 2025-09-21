# Fuzzing
## Build the Image
```bash
docker build -t master-project .
```
Build for ubuntu 20.04
```bash
docker build -f Dockerfile.ubuntu20 -t orb-slam-ubuntu20 .
```

## Run the Container
```bash
docker run -it --name master-project master-project
```

## Run for test and clean up
```bash
docker run --rm -it master-project
```

## Alternative: Run in Detached Mode
```bash
docker run -dit --name master-project master-project
```

## Execute Commands in Running Container
```bash
docker exec -it master-project /bin/bash
```

## What's Included
This Docker container provides:
- AFL++ fuzzing framework
- Rust nightly toolchain with cargo-fuzz
- Pre-built C fuzzing example (`fuzz_simple.c`)
- Pre-configured Rust fuzzing example (`fuzz_safe_copy.rs`)

## Running the Fuzzing Examples

### C Fuzzer
```bash
afl-fuzz -i c-example/in -o c-example/out -- c-example/fuzz_simple @@
```

### Rust Fuzzer
```bash
cd rust-example && cargo fuzz run fuzz_target_1
```

## Clean Up
```bash
# Stop and remove container
docker stop master-project
docker rm master-project

# Remove image
docker rmi master-project
```
