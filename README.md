# robot_chase

run it with 
```
roslaunch robot_chase main.launch
```
and in a separate tab run 
```
roslaunch robot_chase main_function.launch mode:=braitenberg
```
to use the customized environment, set your ENVIRONMENT VARIABLE as:
```
export GAZEBO_MODEL_PATH= your-path-to-robot_chase"robot_chase/models“
```
