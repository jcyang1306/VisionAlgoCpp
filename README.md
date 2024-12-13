# Vision Algo Minimal Pkg

### Build
build pkg in hkclr gsp dev container
```
git clone https://github.com/jcyang1306/VisionAlgoCpp
cd VisionAlgoCpp/

# source ros2 setup bash if necessary
. /opt/ros/foxy/setup.bash

colcon build 
```

### Run
```
. install/setup.bash
./install/vision_algo_cpp/bin/testPclAlgo data/ply_11-41-40.ply
```

