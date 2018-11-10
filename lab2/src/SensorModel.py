#!/usr/bin/env python

import numpy as np
import rospy
import range_libc
import time
from threading import Lock
from nav_msgs.srv import GetMap
import rosbag
import matplotlib.pyplot as plt
import utils as Utils
from sensor_msgs.msg import LaserScan

THETA_DISCRETIZATION = 112 # Discretization of scanning angle
INV_SQUASH_FACTOR = 0.2    # Factor for helping the weight distribution to be less peaked

# YOUR CODE HERE (Set these values and use them in precompute_sensor_model)
Z_SHORT = 0.1 # Weight for short reading
Z_MAX =  0.1 # Weight for max reading
Z_RAND =  0.1 # Weight for random reading
SIGMA_HIT = 0.1 # Noise value for hit reading
Z_HIT =  0.1 # Weight for hit reading

''' 
  Weights particles according to their agreement with the observed data
'''
class SensorModel:
	
  '''
  Initializes the sensor model
    scan_topic: The topic containing laser scans
    laser_ray_step: Step for downsampling laser scans
    exclude_max_range_rays: Whether to exclude rays that are beyond the max range
    max_range_meters: The max range of the laser
    map_msg: A nav_msgs/MapMetaData msg containing the map to use
    particles: The particles to be weighted
    weights: The weights of the particles
    state_lock: Used to control access to particles and weights
  '''
  def __init__(self, scan_topic, laser_ray_step, exclude_max_range_rays, 
               max_range_meters, map_msg, particles, weights, state_lock=None):
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
    self.particles = particles
    self.weights = weights
    
    self.LASER_RAY_STEP = laser_ray_step # Step for downsampling laser scans
    self.EXCLUDE_MAX_RANGE_RAYS = exclude_max_range_rays # Whether to exclude rays that are beyond the max range
    self.MAX_RANGE_METERS = max_range_meters # The max range of the laser
    
    oMap = range_libc.PyOMap(map_msg) # A version of the map that range_libc can understand
    max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution) # The max range in pixels of the laser
    self.range_method = range_libc.PyCDDTCast(oMap, max_range_px, THETA_DISCRETIZATION) # The range method that will be used for ray casting
    #self.range_method = range_libc.PyRayMarchingGPU(oMap, max_range_px) # The range method that will be used for ray casting
    self.range_method.set_sensor_model(self.precompute_sensor_model(max_range_px)) # Load the sensor model expressed as a table
    self.queries = None # Do not modify this variable
    self.ranges = None # Do not modify this variable

    
    arr = []
    i = 0
    minangle = -2.08621382713#msg.angle_min
    maxangle = 2.09234976768 #msg.angle_max
    angleincr = 0.00613592332229 #msg.angle_increment

    for i in range(0, 682):
        arr.append(minangle+angleincr)
        minangle = minangle + angleincr
	i = i +1


    arr = np.asarray(arr)

    self.laser_angles = arr # The angles of each ray
    
    
    self.downsampled_angles = None # The angles of the downsampled rays 
    self.do_resample = False # Set so that outside code can know that it's time to resample
    
    # Subscribe to laser scans
    self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_cb, queue_size=1)    

  '''
    Downsamples laser measurements and applies sensor model
      msg: A sensor_msgs/LaserScan
  '''    
  def lidar_cb(self, msg):
    self.state_lock.acquire()

    #print msg
 
    # Compute the observation obs
    #   obs is a a two element tuple
    #   obs[0] is the downsampled ranges
    #   obs[1] is the downsampled angles
    #   Note it should be the case that obs[0].shape[0] == obs[1].shape[0]
    #   Each element of obs must be a numpy array of type np.float32
    #   Use self.LASER_RAY_STEP as the downsampling step
    #   Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
    #   and vectorizing computations as much as possible
    #   Set all range measurements that are NAN or 0.0 to self.MAX_RANGE_METERS
    #   You may choose to use self.laser_angles and self.downsampled_angles here
    # YOUR CODE HERE

    downsampled_ranges = np.array(np.float32(msg.ranges[0::self.LASER_RAY_STEP]))
    downsampled_angles = np.array(np.float32(self.laser_angles[0::self.LASER_RAY_STEP]))

	#np.array(self.laser_angles[0::self.LASER_RAY_STEP])
    
    #downsampled_angles = np.asarray(downsampled_angles)
    #downsampled_ranges = np.asarray(downsampled_ranges)

    downsampled_angles.astype(np.float32)
    downsampled_ranges.astype(np.float32)

    # NEED TO REPLACE ALL NANS AND ZEROS
    #numpy.num_to_nan()


    if len(downsampled_ranges) == len(downsampled_angles):
        pass
    else:
        print 'DOWNSAMPLE ARRAYS ARE NOT OF SAME LENGTH'

    obs = (downsampled_ranges, downsampled_angles)

    self.apply_sensor_model(self.particles, obs, self.weights)
    self.weights /= np.sum(self.weights)
    
    self.last_laser = msg
    self.do_resample = True
    self.state_lock.release()
  

  '''
  Given ztk and ztk_star (measured and expected ranges, respectively) calculate the probability. 
  '''
  def get_prob(self,ztk, ztk_star):

    # dummy variable; highlights values that are still unknown
    unknown = 1.0
 
    # figure out what ztk, ztk* is
    #ztk = unknown
    #ztk_star = unknown

    # find p_hit
    if 0.0 <= ztk < Z_MAX:
      normal_dist = (1.0/ (2.0*np.pi*(SIGMA_HIT**2.0))**0.5)*(np.exp(-0.5*(((ztk - ztk_star)**2.0)/(SIGMA_HIT**2.0))))
      dztk = 0.0000000001
      normalizer_hit = 1 # instead of normalizing here, we will normalize in the array per row
      p_hit = normal_dist / normalizer_hit 
    else:
      p_hit = 0.0 

    # find p_short
    if 0.0 <= ztk < ztk_star:
      lambda_short = 0.001 # we chose to make lambda_short a very small number
      normalizer_short = 1.0/(1.0- np.exp(-lambda_short*ztk_star))
      p_short =  normalizer_short*lambda_short*np.exp(-lambda_short*ztk)
    else:
      p_short = 0.0

    # find p_max
    if ztk == Z_MAX: # 1 if Z = Z_MAX. 0 otherwise. is it ztk or ztk_star?
      p_max = 1.0 
    else:
      p_max = 0.0

    # find p_rand
    if 0.0 <= ztk < Z_MAX:
      p_rand = 1.0 / Z_MAX 
    else:
      p_rand = 0.0 

    # vector calc of p from textbook is p = Z_array_T dot P_array
    Z_array = np.array([Z_HIT,Z_SHORT,Z_MAX,Z_RAND])
    Z_array_T = np.transpose(Z_array)
    P_array = np.array([p_hit,p_short,p_max,p_rand])
    # p = P(Ztk | xt,m)
    p = np.dot(Z_array_T, P_array)

    return p
 
  '''
    Compute table enumerating the probability of observing a measurement 
    given the expected measurement
    Element (r,d) of the table is the probability of observing measurement r (in pixels)
    when the expected measurement is d (in pixels)
    max_range_px: The maximum range in pixels
    Returns the table (which is a numpy array with dimensions [max_range_px+1, max_range_px+1]) 
  '''  
  def precompute_sensor_model(self, max_range_px):

    table_width = int(max_range_px) + 1
    sensor_model_table = np.zeros((table_width,table_width))

    # Populate sensor_model_table according to the laser beam model specified
    # in CH 6.3 of Probabilistic Robotics
    # Note: no need to use any functions from utils.py to compute between world
    #       and map coordinates here    
    # YOUR CODE HERE
    # Pseudo-code
    # for d in xrange(table_width):
    #   possibly some stuff here
    #   for r in xrange(table_width):
    #     Populate the sensor model table at (r,d) with the probability of 
    #     observing measurement r (in pixels)
    #     when the expected measurement is d (in pixels)
    # Note that the 'self' parameter is completely unused in this function
    

    lambda_short = 0.001

    for d in xrange(table_width):
      normalizer = 0
      for r in xrange(table_width):
	# find p
        p_hit = 1.0/(2.0*np.pi * np.power(SIGMA_HIT,2) ) * np.exp( -0.5*np.power((r-d),2) / np.power(SIGMA_HIT, 2) )
        p_short = float(lambda_short * (np.exp(-lambda_short*r)) ) 
        p_max = (0,1.0)[r==(table_width-1)]
        p_rand = 1.0/max_range_px
        p = Z_HIT*p_hit + Z_SHORT*p_short + Z_MAX*p_max + Z_RAND*p_rand
        sensor_model_table[r,d] = p
        normalizer += p
      sensor_model_table[:,d] /= normalizer
    
    sensor_model_table = sensor_model_table * (1/(table_width)**2) # normalize so all add to 1
    
    return sensor_model_table

  '''
    Updates the particle weights in-place based on the observed laser scan
      proposal_dist: The particles
      obs: The most recent observation
      weights: The weights of each particle
  '''
  def apply_sensor_model(self, proposal_dist, obs, weights):
        
    obs_ranges = obs[0]
    obs_angles = obs[1]
    num_rays = obs_angles.shape[0]
    
    # Only allocate buffers once to avoid slowness
    if not isinstance(self.queries, np.ndarray):
      self.queries = np.zeros((proposal_dist.shape[0],3), dtype=np.float32)
      self.ranges = np.zeros(num_rays*proposal_dist.shape[0], dtype=np.float32)
    
    self.queries[:,:] = proposal_dist[:,:]

    # Raycasting to get expected measurements
    self.range_method.calc_range_repeat_angles(self.queries, obs_angles, self.ranges)

    # Evaluate the sensor model
    self.range_method.eval_sensor_model(obs_ranges, self.ranges, weights, num_rays, proposal_dist.shape[0])

    # Squash weights to prevent too much peakiness
    np.power(weights, INV_SQUASH_FACTOR, weights)


'''
  Code for testing SensorModel
'''

MAP_TOPIC = 'static_map'

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  bag_path = rospy.get_param("~bag_path", '/home/car-user/anptaszn/src/lab2/bags/laser_scans/laser_scan1.bag')
  # /home/car-user/racecar_ws/src/ta_lab2/bags/laser_scans/laser_scan1.bag
  scan_topic = rospy.get_param("~scan_topic", "/scan") # The topic containing laser scans
  laser_ray_step = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
  exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays")) # Whether to exclude rays that are beyond the max range
  max_range_meters = float(rospy.get_param("~max_range_meters")) # The max range of the laser               

  print 'Bag path: ' + bag_path

  # Use the 'static_map' service (launched by MapServer.launch) to get the map
  print("Getting map from service: ", MAP_TOPIC)
  rospy.wait_for_service(MAP_TOPIC)
  map_msg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map # The map, will get passed to init of sensor model
  map_info = map_msg.info # Save info about map for later use    

  print 'Creating permissible region'
  # Create numpy array representing map for later use
  array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
  permissible_region = np.zeros_like(array_255, dtype=bool)
  permissible_region[array_255==0] = 1 # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
                                            # With values 0: not permissible, 1: permissible
  permissible_x, permissible_y = np.where(permissible_region == 1)
  
  # Potentially downsample permissible_x and permissible_y here
  
  print 'Creating particles'
  angle_step = 25
  particles = np.zeros((angle_step * permissible_x.shape[0],3))
  for i in xrange(angle_step):
    particles[i*(particles.shape[0]/angle_step):(i+1)*(particles.shape[0]/angle_step),0] = permissible_y[:]
    particles[i*(particles.shape[0]/angle_step):(i+1)*(particles.shape[0]/angle_step),1] = permissible_x[:]
    particles[i*(particles.shape[0]/angle_step):(i+1)*(particles.shape[0]/angle_step),2] = i*(2*np.pi / angle_step)
  
  Utils.map_to_world(particles, map_info)
  weights = np.ones(particles.shape[0]) / float(particles.shape[0])
  
  print 'Initializing sensor model'
  sm = SensorModel(scan_topic, laser_ray_step, exclude_max_range_rays, 
                   max_range_meters, map_msg, particles, weights)
  
  # Give time to get setup
  rospy.sleep(1.0)
  
  # Load laser scan from bag
  bag = rosbag.Bag(bag_path)
  for _, msg, _ in bag.read_messages(topics=['/scan']):
    laser_msg = msg
    break

  w_min = np.amin(weights)
  w_max = np.amax(weights)
  
  
  pub_laser = rospy.Publisher(scan_topic, LaserScan, queue_size = 1) # Publishes the most recent laser scan
  print("Starting analysis, this could take awhile...")
  while not isinstance(sm.queries, np.ndarray):
    pub_laser.publish(laser_msg)
    rospy.sleep(1.0)
 
  rospy.sleep(1.0) # Make sure there's enough time for laserscan to get lock
  
  print 'Going to wait for sensor model to finish'
  sm.state_lock.acquire()
  print 'Done, preparing to plot'
  weights = weights.reshape((angle_step, -1))
  
  weights = np.amax(weights, axis=0)
  print map_msg.info.height
  print map_msg.info.width
  print weights.shape
  w_min = np.amin(weights)
  w_max = np.amax(weights)
  print 'w_min = %f'%w_min
  print 'w_max = %f'%w_max
  weights = 0.9*(weights-w_min)/(w_max-w_min) + 0.1
  
  img = np.zeros((map_msg.info.height,map_msg.info.width))
  for i in xrange(len(permissible_x)):
    img[permissible_y[i],permissible_x[i]] = weights[i]
  plt.imshow(img)
  plt.show()
  
