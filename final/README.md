# Introduction

The purpose of this project is to autonomously navigate a mobile robot over a series of waypoints as quickly as possible. The robot visits all desired blue waypoints as marked in the real world map and traverses over them while avoiding red waypoints.

Our system design is based on the "line follower", which uses PID control to steer the robot along a provided plan to reach goal pose. We iterated upon the basic functionality of line follower by also implementing a particle filter and computer vision-based control. In this report, we outline the design and implementation of this system in both simulation and reality.

# System and Design


!A high-level diagram of our system.]("https://github.com/annaptasznik/EE545Labs/tree/master/final/project_images/0001.PNG"#center) 

Our system is desined with two different control schemes: the line follower and the vision controller. The highest priority control is vision controller, which is published to /vesc/high_level_ackermann_cmd_mux/input/nav_0. What this means in practive is that if a blue waypoint is detected by the vision controller, it will issue overriding controls to navigate towards the desired waypoint. When the vision controller is idle (no blue is detected), the line follower takes over. Line follower publishes controls to /vesc/high_level_ackermann_cmd_mux/input/nav_1. These control decisions cause the car to always tend towards a predetermined path unless a waypoint is close enough to navigate towards.

These controls rely on information provided by signals coming from the camera and laser scans. Specifically, the camera signal is processed by the vision controller while the laser scan signal is processed by the particle filter. In turn, the particle filter is able to provide a pose estimate to the line follower, which serves as an input to aid in the navigation towards a path. In the meanwhile while all of this is happening, a path is contintuously being published from an offline source.

# Line Follower

Our system relies heavily on the control schemes outlined in the previous line follower lab. The line follower uses PID control to follow a prepublished path. It continually updates the controls based upon the plan index and the inferred pose received from the particle filter.

When the particle filter publishes to the inferred pose the pose_cb() is called. This callback first computes the error between the current pose and the next closest pose in the pose plan array using the compute_error() function. This function returns a flag to indicate if the robot is at the end of the plan and the error.
Using this error then calls compute_steering_angle() which returns a steering delta based on a PID controller algorithm. Once received the callback formats an AckermannDriveStamped message and publishes it to the /vesc/high_level/ackermann_cmd_mux/input/nav_1.

# Particle Filter

The particle filter, the main focus of lab 2, was used to estimate the pose of the robot based on laser scan readings. This pose estimate was published to /pf/viz/inferred_pose and available to other subscribers as a measurement of current position inside the map.

What we refer to as the particle filter is really comprised of four modules--the Motion Model, Sensor Model, Resampler, and Particle Filter. Together, these classes calculate the probability of the robot being in a particular pose. 

## Kinematic Model

The kinematic model estimates control and position noise in our car system by sampling from a Gaussian distribution around the nominal control. This is used to later introduce the noise into the particle filter as it aims to determine its state. The kinematic model subscribes to the motion state topic and servo state topic.

## Sensor Model
The sensor model uses the range_libc library to generate simulated laser observations using raycasting. This simulated observation is then used to calculated the probability of a given pose given the current laser scan. It then updates the weights of the given particles according to this probability.

## Resampler
Resampler uses a low variance sampling algorithm to re sample the particles according to the weights of those particles.

## Particle Filter
The particle filter implements all of the above elements in order to calculate an estimated pose which is then published to the /pf/viz/inferred_pose topic.

# Path Planning and Publishing

We precomputed a path plan using path_planner.py and published it to the /planner_node/full_path_plan node in a separate process on path_publisher.py.

Since the robot’s start, good waypoint, and bad waypoint positions were known in advance, we had enough information to precompute a path which would reach all waypoints most efficiently. The decision to precompute our plan and store it offline allowed us to save onboard computing resources and reduce planning time during the drive. In a different scenario where the course was more dynamic or full of unknown obstacles, a different approach would need to be employed (say, with a method which continuously recomputes best path).

## Path_Planner.py

The purpose of path_planner.py is to generate and store a path for later use. To generate paths, we iterated through each given waypoint, published it as a start pose, published the following waypoint as a goal pose, waited with rospy.sleep(), and published the appended the path plan to a full path plan. To store paths, we saved the full path plan array as a .npy file.
Though we had written a function to automatically calculate poses of each waypoint, we found that it did not result in a straightforward path plan. As a result, we tweaked these poses manually, even adding false waypoints to create plans that involved the fewest loops as possible. The following image shows an example of this:


!A lot of trial and error was involved in picking poses that resulted in straightforward paths.]("https://github.com/annaptasznik/EE545Labs/tree/master/final/project_images/0002.PNG"#center) 

Different plans were tested on the course for accuracy. By trying different paths, we were able to learn and correct for certain problem areas. An example of different generated paths can be seen here:

___INSERT IAMGE___

## Path_Publisher.py

The path publisher’s sole function is to retrieve the stored path and publish it to a node. This message can then be accessed by subscribers in other parts of the system, namely in the line follower. The path publisher takes the .npy file, converts it into a PoseArray message, and continuously publishes it to the /planner_node/full_path_plan node.

# Vision Controller
## Overview

The vision controller detects a colored object (in this case blue) and attempts to steer towards it.
To do this vision controller takes an incoming image from the on-board stereo camera, detects objects that conform to certain specifications and if necessary, publishes control commands in order to keep the object within an area of interest of the camera. 
The  vision controller subscribes to the /camera/color/image_raw topic and publishes to the /camera/color/image_processed and /vesc/high_level/ackermann_cmd_mux/input/nav_0 topics. The vision controller imports from the launch file the parameters listed in the below table. The final value for these parameters is also listed.


___INSERT IMAGE___

An example of the incoming raw image and the processed image is shown below:

___INSERT IMAGE____

## Functions

### main()
The main() function loads the parameters from the launch file. It also calculates a suitable upper and lower bound for the HSV color value corresponding to the input RGB value from the launch file. It then passes all of these parameters into the created vision controller.

### init()
The init() function initializes the node. It creates a subscription to the /camera/color/image_raw topic and two publishers, one to /vesc/high_level/ackermann_cmd_mux/input/nav_0 and one to
/camera/color/image_processed. It also translates the parameters passed from the main() function.

### image_process_cb()
The main part of the vision controller implements a callback image_process_cb() that receives an image message from the /camera/color/image_raw topic. Immediately on receipt of the message the function color_track() is called, which processes the incoming image and produces an x and y pixel position for the detected object. If no object is detected x and y are set to zero. After the image has been processed the resulting the returned x and y position are tested to see if their position corresponds to a region of interest for the controller, these are shown in the below image.


___INSERT IMAGE___

We decided that a y position greater than 360 pixels meant that the object was too close to be considered worth steering towards given our simple control scheme. Similarly, if the x position occurs within the center of the image (between 280 and 360 pixels) it is highly likely that the goal will be reached without any intervention from the vision controller.
If a modification is required to the control this is calculated by the compute_steering() function. This function returns an AckermannDriveStamped message which is published to the
/vesc/high_level/ackermann_cmd_mux/input/nav_0 topic.


### color_track()
The color_track() function takes an incoming message, looks for objects of a certain color and size and outputs the x and y pixel value for the centroid of the object. To do this we first convert the ROS image message into an HSV OpenCV format using the functions CVBridge.imgmsg_to_cv2() and cv2.cvtColor().
Once the message has been converted we create a mask image that displays only pixels that fall within the upper and lower limit required (as calculated in main()). From this mask image we use the cv2.moments() function to calculate the area of the object and if above a given size return the x and y
pixel location of the centroid of the object.
This function also publishes a image message to the topic /camera/color/image_processed that can be used for debug.

### compute_steering()
The compute_steering() function takes the x and y pixel value for the centroid of an object and calculates a steering angle that will keep that object within the area of interest.
This function simply computes the number of pixels the x value is from the center of the image and then
applies a scaling factor to convert this value into a steering angle.
This function returns an AckermannDriveStamped message that can be published as a control.

## Custom Maps
Custom maps were used both to aid in path generation and in the actual robot navigation. In the former case, custom maps were made to simulate barriers or known problem areas that were discovered in testing. For example, plans made in maps without trash bins and stairs lead to collisions with these objects in real life. In this case, the provided maps falsely showed empty space where there were in fact obstacles, so they needed to be corrected. Custom maps were also used to correct areas where obstacles were represented when not in fact there. A good example of this is in the small corridor near the main hall, where furniture was represented in what is actually now an empty space. This proved useful beyond simulation to provide more accurate data to the particle filter.  Examples of these changes are shown below:

___INSERT IMAGE___

# Simulation vs. Robot
## Tuning Parameters
In process of trial and error, we optimized parameters on our physical robot to increase performance.

### Speed
In our observation, higher car speeds resulted in lower waypoint hit accuracy. As such, we slowed our car down to increase our chances of hitting all waypoints successfully. 

___INSERT IMAGE___

### Color
It was necessary to tune the allowable RGB values in our vision controller mask so that it was matching the hues of the waypoint squares. 

# Conclusion
Despite initially being unable to complete the course without intervention (due to striking the bottom stair) we were able to run the entire course in 55.29 seconds hitting all waypoints. Runs at higher speeds also required the same intervention and provided no further benefits. 