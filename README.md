# ROS exercise
This repository is a notebook about trying to solve a proposed exercise using __ROS Noetic__ with a __Polaris gem e2__ robot simulator in __gazebo__ and available __SLAM__ library.

## Exercise tasks
1. Install the POLARIS GEM e2 simulator and apply the noetic joined [patch_gem_simulation](patch/patch_gem_simulation) update.
2. Integrate the LIO-SAM SLAM library into the simulation and apply the joined [patch_liosam](patch/patch_liosam) to handle ROS Noetic.
3. Create a map of the area using LIO-SAM
4. Modify the LIO-SAM to be able to use the generated map instead of building a new one.
    - Implementing a mechanism to load the map into the system at startup. This might involve parsing the map data and initializing certain data structures like a KD-tree.
    - LIO-SAM, as it currently stands, performs scan-matching with recent lidar scans. You would need to modify this to match incoming scans against the pre-built map.
5. Run the modified LIO-SAM using the same pre-built map and compare the results of your localization algorithm with the ground truth from the
gazebo.
6. Build a system for computing an approximative representative metric uncertainty of real-time lidar localization (covariance estimator).
    - Feel free to use any method. In a longitudinal tunnel effect, the vehicle's longitudinal uncertainty should be ideally greater than its lateral error.
    - Evaluate the covariance estimator in relation to the pose error with the ground truth.
7. Prepare the ros package to launch the robot, the world, and your modified localization system.
8. Prepare the dockerfile with all the dependencies in order to run your code and the simulation. Main requirements: ubuntu 20.04, ros noetic and gazebo 11. (A well-written readme would be really appreciated)


## Task solving trials
### Setup the environment
The exercise mentions a deliverable with a Dockerfile and the desired Ubuntu, ROS and gazebo version. So I took the option to immediately begin with having a working Dockerfile.

Fortunately the ***Open Source Robotics Foundation*** responsible of ROS and Gazebo provides [docker images](https://hub.docker.com/u/osrf) and the [`osrf/ros:noetic-desktop-full`](https://hub.docker.com/layers/osrf/ros/noetic-desktop-full/images/sha256-cae9db690397b203c7d000149b17f88f3896a8240bd92a005176460cc73dfe28?context=explore) totally fits the environment specifications.

Then the provided [Dockerfile](Dockerfile) in this repository uses the `osrf/ros:noetic-desktop-full` as a base and is completed with additionnal steps and installation to fullfil Polaris GEM e2 and LIO-SAM requirements.

Since I'm using [vscode](https://code.visualstudio.com/) as an IDE, I found it convenient to use the [dev container extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) then I'm also providing a `.devcontainer.json`. With this extension building the container is a easy a clicking on green bottom left button and choose `Reopen in container`

![remote dev status bar](images/remote-dev-status-bar.png)

Once done you should have the current folder content accessible in the container terminal under the path `/workspace`

While using the container, I faced issue displaying GUI content. This is due to `docker` user by default does not have access to X11 server, to make sure GUI content can be displayed, make sure to run this command from **the host machine**.
```
xhost +local:docker
```
After that, to confirm the setting was up, I've tried to run `roscore` then `rviz` and `gazebo`. Everything were working as expected.

### Install and run the POLARIS GEM e2 simulator
Installing the [POLARIS GEM e2 simulator](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) was straight forward following the official instruction.<br>

Open a terminal in your container and make your ROS worspace directory, in our case `gem_ws`: 
```
mkdir -p /workspace/gem_ws/src
cd /workspace/gem_ws/src
```
Clone the `POLARIS GEM e2 simulator`:
```
git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
```
Apply the simulator patch
```
cd /workspace/gem_ws/src/POLARIS_GEM_e2
git am /workspace/patch/patch_gem_simulation/simu_update.patch
```
Build your ROS workspace
```
cd /workspace/gem_ws
catkin_make
```
Source your workspace setup and you should be able to run the simulator with the following command
```
source /workspace/gem_ws/devel/setup.bash
roslaunch gem_gazebo gem_gazebo_rviz.launch world_name:=/workspace/gem_ws/src/POLARIS_GEM_e2/polaris_gem_simulator/gem_simulator/gem_gazebo/worlds/highbay_track.world x:=-5.5 y:=-21 yaw:=1.5708 velodyne_points:="true" use_rviz:="false"
```
The gazebo simulator should be opened with the robot facing the entrance of a warehouse
![polaris gem gazebo](images/polaris-gem-gazebo.png)


### Teleoperating the POLARIS GEM e2 robot in gazebo
To determine how to operate the robot, I checked the `rostopic list` while the simulation is running and noticed a `/gem/cmd_vel`.<br>
Since the `cmd_vel` is usually the topic to operate robot I've tried to publish the single twist message and could see the robot moving
```
rostopic pub -l /gem/cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0,0]' '[0,0,0]'
```
Then I decide to develop my own keyboard teleoperating script to allow moving the robot. The written script [scripts/gem_teleop_keyboard.py](scripts/gem_teleop_keyboard.py) takes user keyboard inputs to increase/decrease the linear/angular velocities and publish a `geometry_msgs/Twist` to `/gem/cmd_vel`.<br>
To teleoperate the robot, you just need to launch the simulator launch file and from another terminal in container run the written script.
```
python3 /workspace/scripts/gem_teleop_keyboard.py
```
![polaris gem teleop](images/gem-gazebo-keyboard-teleop.gif)
