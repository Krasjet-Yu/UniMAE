# UniMAE

---
## Description
**UniMAE**: An **Uni**fied Framework for **M**ulti-**A**gents **E**xploration

---
## Configuration
1. Install the dependence.
   Install [ompl](https://ompl.kavrakilab.org/), which is required by **car_planner**.
   ```
   sudo apt-get install libompl-dev
   ```

   or install [ompl](https://ompl.kavrakilab.org/) in [site](https://ompl.kavrakilab.org/download.html) .
   ```
   tar zxf ompl-[version].tar.gz
   cd ompl-ompl-*
   ./install-ompl-ubuntu.sh.in
   ```
   
   If occur error like **ompl/base/objectives/PathLengthOptimizationObjective.h : No such file or directory**:
   ```
   sudo apt install ros-noetic-ompl
   sudo find / -name "*ompl*"
   sudo ln -s /opt/ros/noetic/include/ompl-1.4/ompl /opt/ros/noetic/include/ompl
   ```

2. download code
   [Github](https://github.com/Krasjet-Yu/UniMAE.git)
   ```
   git clone https://github.com/Krasjet-Yu/UniMAE.git
   ```

   [Gitee](https://gitee.com/Krasjet_Yu/UniMAE.git)
   ```
   git clone https://gitee.com/Krasjet_Yu/UniMAE.git
   ```

3. compile code
   ```python
   cd UniMAE
   catkin_make # or catkin build
   ```

---
## Class
###  1. Class 1
Build Global Point Cloud Map
```python
source devel/setup.bash
roslaunch map_generator click_map_generator.launch  # generator global map
roslaunch map_generator map_recorder.launch         # record global map
roslaunch map_generator map_publisher.launch        # publish and display global map
```

---
### 2. Class 2
Extract Local Sensing Map (camera and ``lidar``) for Planning Module  
waiting ... 

---
## Run
```bash
source devel/setup.bash && roslaunch scenario run.launch
```

---
## Suggestion
1. whenever you make changes to your code, use the git tool to create a branch of your own
2. when the master branch is updated, switch to the master branch to pull the latest code  
3. merge master branch with your own branch
For example:
```shell
git branch feature-{your name}-class-x     # create a branch named feature-{your name}-class-x
git checkout feature-{your name}-class-x   # switch to your branch
# The above two codes can be combined as: git checkout -b feature-{your name}-class-x
''' modify your code '''
''' when the master branch is updated '''
git merge origin master                    # requires manual conflict resolution
```

---
## Issue
pose_utils not found. Compile pose_utils separately first.
```shell
catkin_make -DCATKIN_WHITELIST_PACKAGES="pose_utils"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""   # cancel separated compile.
```

---
## TODO List
### Simulator
##### Map Generator
- [x] generate click map;
- [x] wapper MapGenerator class for variable buildings
- [ ] operator rqt for map generation
##### Robot Model Simulator 
- [ ] uav simulator
- [x] ugv simulator
- [x] laser simulator
- [ ] camera simulator (add fov)