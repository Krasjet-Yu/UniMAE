# UniMAE

---
## Description
**UniMAE**: An **U**nified Framework for **M**ulti-**A**gents **E**xploration

## Configuration
[Github](https://github.com/Krasjet-Yu/UniMAE.git)
```
git clone https://github.com/Krasjet-Yu/UniMAE.git
```

[Gitee](https://gitee.com/Krasjet_Yu/UniMAE.git)
```
git clone https://gitee.com/Krasjet_Yu/UniMAE.git
```
compile code
```python
cd UniMAE
catkin_make # or catkin build
```

## Class 1
Build Global Point Cloud Map
```python
source devel/setup.bash
roslaunch map_generator click_map_generator.launch  # generator global map
roslaunch map_generator map_recorder.launch         # record global map
roslaunch map_generator map_publisher.launch        # publish and display global map
```

## Class 2
Extract Local Sensing Map (camera and ``lidar``) for Planning Module  
waiting ... 

---
## TODO List
### Simulator
##### Map Generator
- [x] generate click map;
- [ ] wapper MapGenerator class for variable buildings
- [ ] operator rqt for map generation
##### Robot Model Simulator 
- [ ] uav simulator
- [ ] ugv simulator