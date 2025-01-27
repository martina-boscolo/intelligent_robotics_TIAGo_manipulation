# Group 08
- Martina Boscolo Bacheto, martina.boscolobacheto@studenti.unipd.it 
- Tommaso Milanello, tommaso.milanello@studenti.unipd.it 
- Matteo Ruta, matteo.ruta@studenti.unipd.it 

## Instruction Assignment 2
The code for assignment 2 is developed in the ir2425_group_08 package. 
We list here the detail to run the code.
Remember to ```> source ~/catkin_ws/devel/setup.bash``` for each new terminal opened. 

The commands of the first four terminals can be launched at the same time in one terminal with ```> roslaunch ir2425_group_08 Config.launch```.

Also node_a, node_b and node_c can be launched using a launch file, specifically prompting ```> roslaunch ir2425_group_08 Nodes.launch```.

#### Build the packages

```	> cd ~/catkin_ws```

```	> catkin build```
	
```	> source ~/catkin_ws/devel/setup.bash```

#### Terminal 1: Start the simulation 

```	> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment2 ```

#### Terminal 2: AprilTag

```	> roslaunch tiago_iaslab_simulation apriltag2.launch ```

#### Terminal 3: Navigation stack

```	> roslaunch tiago_iaslab_simulation navigation.launch ```

#### Terminal 4: get_straightline_node

```	> rosrun tiago_iaslab_simulation get_straightline_node ```

#### Terminal 5: node_a 

```	> rosrun ir2425_group_08 node_a ```

#### Terminal 6: node_b
```	> rosrun ir2425_group_08 node_b```

#### Terminal 7: node_c
```	> rosrun ir2425_group_08 node_c```

