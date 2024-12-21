# Group 08
- Martina Boscolo Bacheto, martina.boscolobacheto@studenti.unipd.it 
- Tommaso Milanello, tommaso.milanello@studenti.unipd.it 
- Matteo Ruta, matteo.ruta@studenti.unipd.it 

## Instruction HW1
The code for assignment 1 is developed in the ir2425_group_08 package. 
We list here the detail to run the code.
Remember to ```> source ~/catkin_ws/devel/setup.bash``` for each new terminal opened. 

The commands of the first four terminals can be launched at the same time in one terminal with ```> roslaunch ir2425_group_08 Config.launch```

#### Build the packages

```	> cd ~/catkin_ws
	> catkin build
	> source ~/catkin_ws/devel/setup.bash
```

#### Terminal 1: Start the simulation 

```	> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment1 ```

#### Terminal 2: Navigation stack

```	> roslaunch tiago_iaslab_simulation navigation.launch ```

#### Terminal 3: AprilTag

```	> roslaunch tiago_iaslab_simulation apriltag.launch ```


#### Terminal 4: Apriltags IDs server

```	> rosrun tiago_iaslab_simulation apriltag_ids_generator_node ```

#### Terminal 5: node a TODO

```	> rosrun ir2425_group_08 node_a ```

#### Terminal 6: TODO
```	rosrun ir2425_group_08 node_b```
