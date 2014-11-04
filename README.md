##Neural Networks

Neural networks are modeled after how a human brain works. This approach was developed because human brains are very good at processing certain tasks which are challenging for traditional, mathematical based algorythm approaches. Inputs are weighted, then summed at a neuron. Then the neuron puts the result of this through a step function. If the output is over a threshold, if fires off an electrical signal; if not, it does not fire. A good tutorial for understanding neural networks is [here](http://www.theprojectspot.com/tutorial-post/introduction-to-artificial-neural-networks-part-1/7).

## Running the Code

To start ROS and connect to the Neato:
'''bash
roscore 
rosrun neto_node bringup.launch host=[IP_ADDRESS_OF_PI]
```

To take the stored training data (.bag files collected from running the robot the way we want it to run) and use it to train the robot with the latest image processing methods implemented and available:
'''bash

```

To record new training data to teach the robot new behavors, you'll need to pilot the robot via teleop so it does the behavior you want it to do. For instance, if you wanted it to run away from blue cups, you'd need to pilot it so it ran away from blue cups. 4-10 iterations of this is reasonable.
To do this, first run the teleop node:
'''bash 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Then, begin collecting data by running:
'''bash
rosbag ... CYPRESS HLP!!
'''
Note, as soon as collection begins (or as soon as you hit enter in the terminal for this command), you should begin moving the robot. Otherwise, you will need to cut the .bag file recorded by the robot. However, we have implemented functionality which cuts the begining of any .bag file used for training if the velocity is equal to zero. This means that the training data will only begin taking effect once you start moving the robot. 
