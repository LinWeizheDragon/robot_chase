# robot_chase

## Start Environment
run first
```
roslaunch robot_chase main.launch
```
to use the customized environment, set your ENVIRONMENT VARIABLE as:
```
export GAZEBO_MODEL_PATH= your-path-to-robot_chase"robot_chase/models“
```
open RViz using
```
rosrun rviz rviz
```
In ```File - Open Config ``` go to ```robot_chase/rviz/``` open ```velocity_view.rviz```

## Run Experiments
example debug run:
```
roslaunch robot_chase main_function.launch mode:=debug config:=debug max_speed:=0.8,0.8,0.4,0.4 strategy:=estimation visibility:=0,0,0,0
```
example test run:
```
roslaunch robot_chase main_function.launch mode:=test config:=debug max_speed:=0.8,0.8,0.4,0.4 strategy:=estimation visibility:=0,0,0,0 experiment_name:=test_experiment_name num_test:=3 velocity_component:=1,1,1,1,1,1,1,1
roslaunch robot_chase main_function.launch mode:=test config:=2police2baddy max_speed:=0.8,0.8,0.4,0.4 strategy:=estimation visibility:=0,0,0,0 experiment_name:=2police2baddy_estimation_test num_test:=3 velocity_component:=1,1,1,1,1,1,1,1
```
### Arguments
```
--mode:
    debug: not saving experiment data, run only once
    test: run multiple experiments
--config:
    config name. Add new configs in config.py
--max_speed:
    none: default - use values in config
    [int, int, ...]: list of speed of robots. Assign in order.
--strategy:
    naive: chase the absolute position of baddies
    estimation: chase the estimated pose
--visibility:
    none: default - use values in config
    [int, int, ...]: list of visibility of robots. Assign in order. -1 for full vision.
--experiment_name:
    path name to save results
--num_test:
    int. number of experiments to run
--velocity_component:
    none: default - use values in config
    [int, int, ...]: list of amplification on each velocity component. <br />
    Assign in order, chase_baddy, avoid_corners, avoid_hitting_obstacles, <br />
    avoid_hitting_police, avoid_hitting_baddies, avoid_hitting_captured_baddies, <br />
    avoid_hitting_walls, escape_from_police
```
