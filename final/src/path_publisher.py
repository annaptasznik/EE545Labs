#!/usr/bin/env python

import os.path
import rospy
import numpy as np
import utils

from geometry_msgs.msg import PoseArray


# full car plan (start point to furthest waypoint, all waypoints touched) 
PLAN_PUB_TOPIC = "/planner_node/full_path_plan"

if __name__ == "__main__":

    # set up publisher and node
    plan_pub = rospy.Publisher(PLAN_PUB_TOPIC, PoseArray, queue_size=10)
    rospy.init_node("PathPublisher", anonymous=True)

    # file storing plan poses
    offline_plan_poses = '/home/car-user/anptaszn/src/final/offline_data/output_plan.npy'

    if os.path.isfile(offline_plan_poses):
      
      print "offline plan found"
      offline_plan = np.load(offline_plan_poses)
      path_plan = offline_plan

    else:
      print "no offline plan found"
    
    
    rospy.sleep(4)

    # create and publish the plan message 
    plan_pa_msg = PoseArray()
    plan_pa_msg.header.frame_id = "/map"
    plan_pa_msg.poses = path_plan
    plan_pub.publish(plan_pa_msg)
    print "published the plan."


    rospy.spin()
