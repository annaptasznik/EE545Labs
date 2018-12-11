#!/usr/bin/env python

import os.path
import rospy
import csv
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
import utils


# full car plan (start point to furthest waypoint, all waypoints touched) 
PLAN_PUB_TOPIC = "/planner_node/full_path_plan"

# start/end poses for each waypoint 
INITIAL_PUB_TOPIC = "/initialpose"
GOAL_PUB_TOPIC = "/move_base_simple/goal"
MAP_TOPIC = "static_map" 
WP_VIZ_TOPIC = "/planner_node/waypoints"

class PathPlanner:

    def __init__(self, plan, waypoints):
        self.plan = plan
        self.waypoints = waypoints

        self.plan_pub = rospy.Publisher(PLAN_PUB_TOPIC, PoseArray, queue_size=10)
        self.initial_pub = rospy.Publisher(INITIAL_PUB_TOPIC, PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub = rospy.Publisher(GOAL_PUB_TOPIC, PoseStamped, queue_size=1)

        self.waypoint_pub = rospy.Publisher(WP_VIZ_TOPIC, PoseArray, queue_size=1)
        
    '''
    Test function used for to visualize waypoint poses.
    '''
    def print_waypoints(self, waypoints):
      WpMsg = PoseArray()
      WpMsg.header.frame_id = "/map"
      wp_plan = []
      for waypoint in waypoints:
        x = waypoints[0]
        y = waypoints[1]
        quaternion = utils.angle_to_quaternion(waypoints[2])
        wp_plan.poses.orientation = quaternion
        wp_plan.poses.pose.x =  x 
        wp_plan.poses.pose.y = y
        wp_plan.extend(wp_plan.poses)  
      WpMsg.poses = wp_plan
      waypoint_sub.publish(WpMsg)
        
    '''
    Given two poses, publish them as a start and goal pose
    to the appropriate topics.
    '''
    def get_segment_plan(self, start_pose, waypoint_pose, time_stamp):     

        start_x,start_y ,start_theta = start_pose[0],start_pose[1], start_pose[2]
        wp_x,wp_y,wp_theta = waypoint_pose[0], waypoint_pose[1], waypoint_pose[2]

        # set up and publish the start pose startMsg
        startMsg = PoseWithCovarianceStamped()
        quaternion = utils.angle_to_quaternion(start_theta)
        startMsg.header.stamp = time_stamp
        startMsg.header.frame_id = "/map"
        startMsg.pose.pose.position.x = start_x
        startMsg.pose.pose.position.y = start_y
        startMsg.pose.pose.orientation = quaternion

        # set up and publish the goal pose goalMsg
        goalMsg = PoseStamped()
        quaternion = utils.angle_to_quaternion(wp_theta)
        goalMsg.header.stamp = time_stamp 
        goalMsg.header.frame_id = "/map"
        goalMsg.pose.position.x = wp_x
        goalMsg.pose.position.y = wp_y
        goalMsg.pose.orientation = quaternion

        self.initial_pub.publish(startMsg) 
        self.goal_pub.publish(goalMsg)

'''
Return an ordered waypoints array listed in the order they should be visited. Order is chosen based on which waypoint is closest. 
'''
def order_waypoints(start_path, good_waypoints_path):

  good_waypoints = csv_to_arr(good_waypoints_path)
  start = csv_to_arr(start_path)[0]

  ordered_wp= []
  ordered_wp.append([start[0], start[1]])

  size = len(good_waypoints)+1

  for x in xrange(21):
    min_dist = 10000
    if good_waypoints != []:
      
      for waypoint in good_waypoints:
        start_x, start_y = start[0], start[1]
        goal_x, goal_y = waypoint[0], waypoint[1]
        dist = np.sqrt((goal_x - start_x)**2 + (goal_y-start_y)**2)

        # if waypoint is closer than the last, save it
        if dist < min_dist:
          min_dist = dist
          lowest_x, lowest_y = goal_x, goal_y

      ordered_wp.append([lowest_x,lowest_y])

      try:
        good_waypoints.remove([start[0], start[1]]) 
      except:
        pass
      start = [lowest_x, lowest_y]
  uniquelist = []
  for elem in ordered_wp:
    if elem not in uniquelist:
      uniquelist.append(elem)

  return uniquelist

'''
Find orientations of each waypoint pose. Should be a vector pointing towards next waypoint.
'''    
def calc_waypoint_orientation(ordered_list):
  full_waypoint_data = []
  i = 0
  while i < (len(ordered_list)-1):
    x = ordered_list[i][0]
    y = ordered_list[i][1]

    nextx = ordered_list[i+1][0]
    nexty = ordered_list[i+1][1]
    theta = np.arctan2((nextx-x),(nexty-y))
    full_waypoint_data.append([x,y,theta])
    i = i +1

  return full_waypoint_data

'''
Read csv file into array
'''
def csv_to_arr(csv_file):
  res = []
  with open(csv_file) as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
      try:
        row[0] = int(row[0])
        row[1] = int(row[1])
        res.append(row)
      except:
        pass
  return res

'''
Custom map_to_world function (from utils) which flips pose
Logic taken from another team's github code:
github.com/russellguzman/ee545_robot_car
'''
def map_to_world(poses,map_info):

    pose[:,1]=2*618-pose[:,1]

    scale = map_info.resolution
    angle = utils.quaternion_to_angle(map_info.origin.orientation)

    # Rotation
    c, s = np.cos(angle), np.sin(angle)
    
    # Store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp      + c*poses[:,1]

    # Scale
    poses[:,:2] *= float(scale)

    # Translate
    poses[:,0] += map_info.origin.position.x
    poses[:,1] += map_info.origin.position.y
    poses[:,2] += angle

if __name__ == "__main__":

    rospy.init_node("PathPlanner", anonymous=True) 

    # Default values
    plan_topic = "/planner_node/car_plan"



    good_waypoints_path = "/home/car-user/anptaszn/src/final/waypoints/real_car/good_waypoints.csv"
    bad_waypoints_path = "/home/car-user/anptaszn/src/final/waypoints/real_car/bad_waypoints.csv"
    start_path = "/home/car-user/anptaszn/src/final/waypoints/real_car/start.csv"

    # set up waypoints and poses automatically; this does not always give
    # the most straightforward paths, so customized poses were used 
    # instead 

    #waypoints = calc_waypoint_orientation(order_waypoints(start_path, good_waypoints_path))

    start_pose = np.array([[2500, 640, 6.0]])

    waypoints = np.array([[2600, 660, 6.5],
                            [2600, 450, 2.8],
			    [1880, 440, 3.0],
			    [1699, 450, 4.5],
                            [1590, 670, 3.3],
                            [1490, 570, 1.85], 
                            [1430, 490, 2.7],
                            [1250, 460, 3.0], 
                            [1150, 460, 3.2 ],
                            [950, 480, 3.6 ],
                            [600, 700 ,3.9],
                            [540, 835, 4.8 ]])

    plan=[]
    map_img, map_info = utils.get_map(MAP_TOPIC)

    map_to_world(start_pose,map_info)
    map_to_world(waypoints,map_info)

    pp = PathPlanner(plan,waypoints)

    # offline plan location
    offline_plan_path = '/home/car-user/anptaszn/src/final/offline_data/output_plan.npy'

    # check if offline plan already exists. if yes, do not 
    # continue
    if os.path.isfile(offline_plan_path):
      
      print "offline plan found"
      offline_plan = np.load(offline_plan_path)
      plan = offline_plan
      

    else:
      print "no offline plan found: a new plan will be computed"

      # loop over waypoints to get path plans between them
      start_pose = start_pose[0,:]
      for waypoint_pose in waypoints: 

          string = str(waypoint_pose[0])+str(waypoint_pose[1])
          rospy.sleep(5)

          pp.get_segment_plan(start_pose, waypoint_pose, rospy.get_rostime())
          
          print "waiting for a plan..."
	  rospy.sleep(30)

          # add segment plan to the plan
          plan_msg = rospy.wait_for_message(plan_topic, PoseArray)
          plan.extend(plan_msg.poses)
          start_pose = waypoint_pose

    
    np.save(offline_plan_path,plan)
    print "done planning."


    rospy.sleep(5)
    poseArrayMsg = PoseArray()
    poseArrayMsg.header.frame_id = "/map"
    poseArrayMsg.poses = plan
    pp.plan_pub.publish(poseArrayMsg) 

    rospy.spin()  
