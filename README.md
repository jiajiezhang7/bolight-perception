# Perception

## How to Use Perception

### Environment Setup

#### Third Party: yaml-cpp

```
sudo apt update
sudo apt install libyaml-cpp-dev
```



#### Third Party: gemographicLib

```
git clone https://github.com/geographiclib/geographiclib
cd geographiclib/
mkdir build
cd build
cmake ..
make
sudo make intall
```





1. Create a workspace:

```
mkdir -p workspace/src
cd workspace
catkin_make
```



if enconter error when 'catkin_make', you shall try:

`catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`


2. under your workspace/src, `git clone ssh://git@star-center.shanghaitech.edu.cn:10022/bolight-mars/perception.git`

3. Download the dataset to your computer: Bolight/3lidars_stands.bag from the seafile website.

4. In perception/launch/action.launch, change the default bag file path into where you save 3lidars_stands.bag. 

5. In perception/config/file_config.yaml, change file path: /home/xxx/workspace/src/perception/data/stands_model_pc.pcd



### Run Program
1. To run the program, (under workspace):

```
catkin_make
"source ./devel/setup.bash"
"roslaunch perception action.launch"
```



check action.launch, if you want Perception part as an action server, you shall comment a line of code (action_client) in action.launch
