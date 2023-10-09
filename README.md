# ICP Odometry
1. cd into folder

```bash
cd ~/ELEC3210_projects/P1_ICP
```

2. Compile the code

```bash
catkin_make
```

3. Source the setup file

```bash
source devel/setup.bash
```

4. Run the code

* In terminal 1, run the following command

```bash
roscore
```

* In terminal 2, run the following command

```bash
roslaunch icp_odometry icp.launch
```
