# Fuzzing
## Build the Image
```bash
docker build -t orb-slam-ubuntu20 .
```

## Run the Image
```bash
docker run -it dreamking60/orb-slam3-ubuntu20 --name test_orb /bin/bash
```

```bash
docker exec -it test_orb /bin/bash
```

## ROS Run test
First run the roscore in a terminal.
```bash
roscore
```

Then run the orb-slam3 package in another terminal.
```bash
cd /home/orb-slam-elastic
rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt /home/orb-slam-elastic/Examples_old/Stereo-Inertial/EuRoC.yaml true &
```

Finally run the rosbag replay in another terminal.
```bash

```

### ORB_SLAM3 Package
Maybe can't find the package, so we need to source the setup.bash file.
```bash
source /home/orb-slam-elastic/Examples_old/ROS/ORB_SLAM3/build/devel/setup.bash
```


## My version
I have build my own version of docker at `dreamking60/orb-slam3-ubuntu20`.
