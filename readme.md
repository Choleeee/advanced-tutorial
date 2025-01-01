## Install VRX workspace

```
Follow this website (remember to also install gazebo garden)
https://github.com/osrf/vrx/wiki/tutorials 
```


## Task: Circle a point (the white object) and maintain the distance between the ASV (Boat) and the point within 5~6 m.

How to control the boat
- By publishing data (thrust and angle) to the related topics 

Steps: 
1. Create a package under your workspace/src
2. Copy the following skeleton code into the respective files (remember to change the package name/executable names accordingly)

Submission:
Create a repository containing your package

--- 
## To run the simulator:
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task

## Customizing own wamv
For better control, you may consider creating your own boat (customize its thruster positions)
Follow these steps: https://github.com/osrf/vrx/wiki/customizing_wamv_beginner_tutorial

To run the simulator (with a customized boat):
Starting from your workspace:
```
source /opt/ros/humble/setup.bash
cd ~/vrx_ws 
colcon build --merge-install
. install/setup.bash
```
Then `cd ~/my_wamv`
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task urdf:=`pwd`/wamv_target.urdf

---
## Running the package (Aka controlling the boat in the simulator
Then, go to the command terminal and open another window, cd the workspace with your package (remember to source the workspace with /opt/ros/<distro>/setup.bash and source install/local_setup.bash)

Then run your package 
`ros2 run <package name> <executable>`

--- 
## Topics when the simulator is running:

(Hint: Run `ros2 topic list` when running the simulator)
(Hint2: Be careful with the GPS and IMU  + find the relevant equations online ( e.g distance between to GPS coordinates and z axis angle ))

| Topic                   | Data Type | Description |
| :---------------- | :------ | :---- |
| /vrx/stationkeeping/goal        | geometry_msgs::msg::PoseStamped  | Position of the point in spherica; (WGS84) coordinates and a heading, given as a quaternion |
| /wamv/sensors/gps/gps/fix |  sensor_msgs::msg::Imu  | IMU data of the boat |
| /wamv/sensors/gps/gps/fix |  sensor_msgs::msg::NavSatFix  | GPS position data |
|   /wamv/thrusters/<thrustername>/pos  |  std_msgs::msg::Float64   | Next angle command for the <thrustername> thruster |
|  /wamv/thrusters/<thrustername>/thrust |  std_msgs::msg::Float64   | Next power command for the <thrustername> thruster|



## Possible errors:
