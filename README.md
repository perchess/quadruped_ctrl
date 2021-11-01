# MIT convex MPC library

### Based on [quadruped_ctrl](https://github.com/Derek-TH-Wang/quadruped_ctrl "Derek-TH-Wang's Repo") repo
The package changed the types of arguments and edited the ros messages under the c++ format

There will be a gif here

### System requirements:
Ubuntu 20.04, ROS Melodic

### Dependency:
use Logitech gamepad to control robot
```
git clone https://github.com/Derek-TH-Wang/gamepad_ctrl.git
```

### Build
```
cd {your workspace}
catkin make
source devel/setup.bash
```

#### Install Python dependencies

```bash
pip3 install -r requirements.txt
```

### Terrain
you can modify the ```config/quadruped_ctrl_cinfig.yaml/terrain``` to deploy different terrains, there are four terrains supported in the simulator now, for example:
```
"plane"
"stairs"
"random1"
"random2"
"racetrack"
```

### Running:
run the gamepad node to control robot:
```
roslaunch gamepad_ctrl gamepad_ctrl.launch
```
run the controller in simulator:
```
roslaunch quadruped_ctrl quadruped_ctrl.launch
```

switch the camera on / off:
camera set ```True``` or ```False``` in ```config/quadruped_ctrl_config.yaml```, then launch the rviz to see the point cloud:
```
roslaunch quadruped_ctrl vision.launch
```

also can switch the gait type:
```
rosservice call /gait_type "cmd: 1"
```

gait type:
```
0:trot
1:bunding
2:pronking
3:random
4:standing
5:trotRunning
6:random2
7:galloping
8:pacing
9:trot (same as 0)
10:walking
11:walking2
```

