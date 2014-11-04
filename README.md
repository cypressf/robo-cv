## Running the Code

### Start ROS and connect to the Neato.
```bash
roscore 
rosrun neato_node bringup.launch host=[IP_ADDRESS_OF_PI]
```

### Pilot the robot via teleop in the behavior you want it to mimic.
For instance, if you want it to run away from blue cups, you'd need to pilot it so it ran away from blue cups. 4-10 iterations of this is reasonable.

Do this with a joystick controller to control the robot with smooth turns.
```bash 
rosrun joy joy_node
rosrun cv_project teleop.py
```

Or if you don't have access to a joystick controller.
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Begin collecting data (hit ctl-c to stop).
```bash
rosbag record /scan /camera/image_raw/compressed /odom /tf /tf_static /cmd_vel
```
Note, as soon as collection begins (or as soon as you hit enter in the terminal for this command), you should begin moving the robot. Otherwise, you will need to cut the .bag file recorded by the robot. However, we have implemented functionality which cuts the begining of any .bag file used for training if the velocity is equal to zero. This means that the training data will only begin taking effect once you start moving the robot.


### Use the .bag files to train the ridge regression.
```bash
python cv_trainer.py [PATH_TO_BAG_FILES_DIRECTORY]
```
When it's done processing, it will prompt you for a file to save your ridge regression, so you can use it to control the robot!

### Control the robot with your new ridge regression.
```bash
rosrun cv_project cv_follower.py [PATH_TO_RIDGE_REGRESSION_SAVE_FILE]
```

## How it works: neural Networks

Neural networks are modeled after how a human brain works. This approach was developed because human brains are very good at processing certain tasks which are challenging for traditional, mathematical based algorythm approaches. Inputs are weighted, then summed at a neuron. Then the neuron puts the result of this through a step function. If the output is over a threshold, if fires off an electrical signal; if not, it does not fire. A good tutorial for understanding neural networks is [here](http://www.theprojectspot.com/tutorial-post/introduction-to-artificial-neural-networks-part-1/7).
