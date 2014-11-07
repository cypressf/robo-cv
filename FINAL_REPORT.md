# Computer Vision Robots Project
Red Solo Cup + Blue Trash Can  
Cypress Frankenfeld  
Halie Murray-Davis  
Nov 7 2014  
## Our goal
Train the robot to move towards an object using the camera, opencv, and a learning model in scikit learn. We first tried to get our robot to move towards a blue trashcan, then a red cup. We also did very limited work getting the robot to move toward blue cups; however, this work is far from complete. 
## How we solved the problem
We recorded training data by driving the robot towards our object from various starting positions and saving the images and velocity vectors to a rosbag file. In the first iterations of using computer vision to make our target object more obvious, in this case, a red Solo cup, we used a threshold to filter out everything but the red cup.

![](/documentation_images/images.png)

Once we masked the image, added up the column pixel intensity values to get a one-dimensional vector. This vector was our first iteration at representing the image. We used it in combination with the velocity vector at the time the image was taken to train a ridge regression. We then used the ridge regression to estimate velocity vectors given new processed images. This worked extremely well. We also showed that a similar approach would work for detecting and navigating toward dark blue trashcans. 

As we further developed and refined our methods, we began to filter the images before applying the threshold. The reason for this is that the lights in our testing room tended to create a lot of glare. Since we applied a color threshold, this apparent variation in the color of the target object yielded less than desirable results. The ridge regression was able to account for strangely shaped contours caused by glare, but in the interest of continual improvement, we sought to remove them. 
## Design decision
We experimented with several methods for masking the image, and finding the location of our target object. We tried using contours to isolate it, but we ended up going with a color threshold because the objects that we were using (a blue trashcan and a red cup) had uniform colors that stood out from the background.
## How we structured our code
We created three main files:
`cv_trainer.py` a script that converts training data to a saved ridge regression
`cv_follower.py` ros node that loads a saved ridge regression and controls the robot
`image_processing.py` the image processing code that is shared between trainer and follower

`cv_trainer.py` is a simple python script. To run it, you pass a path to a directory that contains training bag files, and you give it a filename to save a cached ridge regression. It loops through the images and `cmd_vel` messages in the bag files, runs the `extract_data(cv_image)` function from `image_processing.py` file on each image, and then runs `fit(image_data, cmd_vel_data)` on a ridge regression, and saves the ridge regression in a pickled python file.

`cv_follower.py` contains the `Controller` class, which we use to control our neato. It loads a saved ridge regression. `Controller` subscribes to camera/image_raw. It runs extract_data(cv_image) on images as they arrive, passes the extracted vector to the ridge regression to estimate the `cmd_vel`, and then publishes the estimated cmd_vel to drive the neato.

`image_processing.py` contains a function called `extract_data(cv_image)`, which takes a two-dimensional image array and returns a one-dimensional array that represents the relevant data we extract from the image using our image processing algorithm. We created several different image processing algorithms and swapped out which one `extract_data` called, depending on what we wanted to test.
## What if any challenges did you face along the way?
### Rosbag files contained extraneous footage wherein the robot wasn’t moving
When we recorded rosbag files, we had to start the robot moving immediately, and stop the recording before the robot stopped in order to avoid contaminating our training data with zero-vector cmd_vel messages. To extract relevant training data, we cropped out the zero-vector cmd_vel messages at the beginning and end of our rosbag files with some if-statements in cv_trainer.py.
### We needed to find a way to represent our image as a one-dimensional vector
The scikit learn ridge regression accepts only one-dimensional vector inputs, and our images are two-dimensional. Our initial solution outlined in the “How we solved the problem” (adding up the columns of our image and normalizing) only captured the location of the target object along the horizontal axis, but didn’t capture its position along the vertical axis.
## If we had time, we would… 
### Properly tune the alpha parameter of the ridge regression
We would be able to tune it by training on one set of data, then testing it on another set, finding the difference between our estimate and the real data, then minimizing the difference.
### Use a Support Vector Machine to avoid the constraints of the ridge regression
The main problem with the ridge regression as our model is that it can only find estimates based on a linear combination of the input vector. A support vector machine would enable us to generate a nonlinear model.
### Gather better training data
Some of our training data was not very well recorded. We drove the robot using a joystick to make the behavior of the velocity vector a little more smooth, but we still had issues overshooting the target when turning (just due to human error), so the training data wasn’t optimal.
### Create a visualization of the predictions and compare it to some test data
We could more easily iterate and test our model and image processing by comparing estimated velocity vectors to actual velocity from our validation data.

![](/documentation_images/vectors.png)

## Lessons we learned for future robotics projects
### You can iterate through rosbag files using a python API
There is no need to use the command line interface and rate. There’s an API that turns rosbag files into python iterables. It’s really fast and easy to use, especially when using bagged data for statistical analysis or machine learning training.
### For simple problems, naive machine learning isn’t as good at controlling the robot as a handmade algorithm would be
Our goal, to move to an object, could have been accomplished with a simple proportional control algorithm that looks at the position of the object in the camera, and how large it is to determine which way to turn, and how fast to go. Machine learning was a good learning experience for us, but we probably could have made a better algorithm in a shorter amount of time on our own.
### Lighting matters a lot in computer vision, especially if you’re using a threshold color
The threshold we set initially for the blue trashcan didn’t hold up when the room was darker. We had to adjust the lighting to be similar to the lighting that we used when we recorded it.
